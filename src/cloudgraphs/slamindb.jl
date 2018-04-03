# SLAMinDB service functions

function getcredentials(;nparticles=true, drawdepth=true, multisession=false, drawedges=true)
  cloudGraph, addrdict = standardcloudgraphsetup(nparticles=nparticles, drawdepth=drawdepth, drawedges=drawedges, multisession=multisession)
  return addrdict
end

function slamindbsavejld(fgl::FactorGraph, session::AbstractString, itercount::Int)
  dt = Base.Dates.now()
  filenamejld = "$(session)_$(Dates.format(dt, "dduyy-HH:MM:SS"))_slamindb_$(itercount).jld"
  println("------Save fg to file: $(filenamejld)------")
  savejld(fgl,file=filenamejld)
  nothing
end

"""
Get the command-line parameters.
"""
function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        "sysConfig"
            help = "Provide a system configuration file"
            default = "./config/systemconfig.json"
            arg_type = String
        "executionMode"
            help = "Either webserver or finiteiterations - if finiteiterations, then a session and iteration count must be specified"
            arg_type = String
            range_tester = (op->lowercase(op)=="webserver"||lowercase(op)=="finiteiterations")
            default = "webserver"
        "--session", "-s"
            help = "The session name, required if running as finitesessions"
        "--iterationCount", "-i"
            arg_type = Int64
            help = "The number of iterations to run against the session if using finiteiterations mode of operation"
    end

    return parse_args(s)
end

"""
    runWebServer(sysConfig, cloudGraph)
Run the blocking webserver that allows a user to control Caesar.
Probably will be moved to an independent file in future.
"""
function runWebServer(sysConfig, cloudGraph)
    webconfig = sysConfig.caesarConfig.webserverConfig

    # funcs = Dict{Any, Dict{String, Any}}(
    #     r"^/status" => Dict{String, Any}(
    #         "GET" => return "UP"
    #     )
    # )

    http = HttpHandler() do req::Request, res::Response
        @show req
            Response( ismatch(r"^/status",req.resource) ? string("Hello ", split(req.resource,'/')[3], "!") : 404 )
    end
    http.events["error"]  = (client, err) -> println(err)
    http.events["listen"] = (port)        -> println("Listening on $port...")

    server = Server( http )
    run( server, webconfig.port )

    println("Exiting!")
end

"""
    startSlamInDb()
Main function for SlamInDb - arguments are parsed from command-line, pass it
a sysConfig path argument.
"""
function startSlamInDb()
    # 1. Parse command lines.
    parsedArgs = parse_commandline()
    execMode = parsedArgs["executionMode"]

    # 2. Read system config.
    println(" --- Reading system config file $(parsedArgs["sysConfig"])...")
    sysConfig = readSystemConfigFile(parsedArgs["sysConfig"])

    println(" --- Initiating CloudGraphs connection to '$(sysConfig.cloudGraphsConfig.neo4jHost)' and '$(sysConfig.cloudGraphsConfig.mongoHost)'...")
    cloudGraph = connect(sysConfig.cloudGraphsConfig);

    println(" --- Registering Cloudgraphs types...")
    registerGeneralVariableTypes!(cloudGraph)
    Caesar.usecloudgraphsdatalayer!()

    # 3. If finite run, run it, otherwise start the webserver.
    if(execMode == "webserver")
        println(" --- Caesar is configured for continuous operation, starting webserver on $(sysConfig.caesarConfig.webserverConfig.port)!")
        runWebServer(sysConfig, cloudGraph)
    else
        sessionName = parsedArgs["session"]
        iterationCount = parsedArgs["iterationCount"]
        println(" --- Caesar is configured for a finite run of $iterationCount iterations on session '$sessionName'...")
        # runSlamInDbOnSession(sysConfig.caesarConfig, cloudGraph, sessionName, iterationCount)
        runDbSolver(cloudGraph,
                    sessionName,
                    N=sysConfig.caesarConfig.numParticles,
                    iterations=iterationCount,
                    multisession=sysConfig.caesarConfig.multisession)
    end
end

"""
Low-level call to iterate the SlamInDb solver for given number of iterations against a specific session and keyword parameters.
"""
function runDbSolver(cloudGraph::CloudGraphs.CloudGraph,
            sessionName::A;
            N::Int=100,
            loopctrl::Vector{Bool}=Bool[true],
            iterations::Int=-1,
            multisession::Vector{A}=Sting[""],
            savejlds::Bool=false,
            recursivesolver::Bool=false,
            drawbayestree::Bool=false  ) where {A <: AbstractString}

  itercount = 0
  while loopctrl[1] && (iterations > 0 || iterations == -1) # loopctrl for future use
    iterations = iterations == -1 ? iterations : iterations-1 # stop at 0 or continue indefinitely if -1
    println("===================CONVERT===================")
    fgl = Caesar.initfg(sessionname=sessionName, cloudgraph=cloudGraph)
    updatenewverts!(fgl, N=N)
    println()

    println("=============ENSURE INITIALIZED==============")
    ensureAllInitialized!(fgl)
    println()

    println("================MULTI-SESSION================")
    rmInstMultisessionPriors!(cloudGraph, session=sessionName, multisessions=multisession)
    println()

    println("====================SOLVE====================")
    fgl = Caesar.initfg(sessionname=sessionName, cloudgraph=cloudGraph)

    setBackendWorkingSet!(cloudGraph.neo4j.connection, sessionName)

    println("get local copy of graph")

    if fullLocalGraphCopy!(fgl)
      (savejlds && itercount == 0) ? slamindbsavejld(fgl, sessionName, itercount) : nothing
      itercount += 1

      println("-------------Ensure Initialization-----------")
      ensureAllInitialized!(fgl)

      println("------------Bayes (Junction) Tree------------")
      tree = wipeBuildNewTree!(fgl,drawpdf=drawbayestree)
      if !recursivesolver
        inferOverTree!(fgl, tree, N=N)
      else
        inferOverTreeR!(fgl, tree, N=N)
      end
      savejlds ? slamindbsavejld(fgl, sessionName, itercount) : nothing
    else
      sleep(0.2)
    end
  end
end

# """
#     runSlamInDbOnSession(sysConfig::SystemConfig, cloudGraph::CloudGraph)
# Runs SlamInDb for given number of iterations against a specific session.
# """
# function runSlamInDbOnSession(caesarConfig::CaesarConfig,
#             cloudGraph::CloudGraph,
#             sessionName::String,
#             iterations::Int64;
#             loopctrl::Vector{Bool}=Bool[true],
#             savejlds::Bool=false,
#             recursivesolver::Bool=false,
#             drawbayestree::Bool=false  )::Void
#
#     runDbSolver(cloudGraph,
#                 sessionName,
#                 N=caesarConfig.numParticles,
#                 loopctrl=loopctrl,
#                 iterations=iterations,
#                 multisession=multisession,
#                 savejlds=savejlds,
#                 recursivesolver=recursivesolver,
#                 drawbayestree=drawbayestree)
# end

"""
Manually call the SLAMinDB solver to perform inference on a specified session given the keyword parameter settings.
"""
function slamindb(;addrdict=nothing,
            N::Int=-1,
            loopctrl::Vector{Bool}=Bool[true],
            iterations::Int=-1,
            multisession::Bool=false,
            savejlds::Bool=false,
            recursivesolver::Bool=false,
            drawbayestree::Bool=false  )


  nparticles = false
  if N > 0
    addrdict["num particles"] = string(N)
  else
    nparticles = true
  end

  cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict,
          nparticles=nparticles, multisession=multisession)

  N = parse(Int, addrdict["num particles"])
  session = addrdict["session"]

  if !haskey(addrdict, "multisession")
    addrdict["multisession"]=String[]
  end

  runDbSolver(cloudGraph,
              session,
              N=N,
              loopctrl=loopctrl,
              iterations=iterations,
              multisession=addrdict["multisession"],
              savejlds=savejlds,
              recursivesolver=recursivesolver,
              drawbayestree=drawbayestree)
  nothing
end


function convertdb(;addrdict=nothing,
      N::Int=-1  )
  #
  nparticles = false
  if N > 0
    addrdict["num particles"] = string(N)
  else
    nparticles = true
  end
  cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict, nparticles=nparticles)
  N = parse(Int, addrdict["num particles"])
  fg = Caesar.initfg(sessionname=addrdict["session"], cloudgraph=cloudGraph)
  updatenewverts!(fg, N=N)
end

function resetconvertdb(;addrdict=nothing)
  cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)
  println("Clearing slamindb data, leaving front-end data, session: $(addrdict["session"])")
  resetentireremotesession(cloudGraph.neo4j.connection, addrdict["session"])
end























#
