# SLAMinDB service functions
using Base.Dates

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
    runSlamInDbOnSession(sysConfig::SystemConfig, cloudGraph::CloudGraph, userId::String, robotId::String, sessionId::String, iterations::Int64, solverStatus::SolverStatus)
Runs SlamInDb for given number of iterations against a specific session.
"""
function runSlamInDbOnSession(caesarConfig::CaesarConfig, cloudGraph::CloudGraph, userId::String, robotId::String, sessionId::String, iterations::Int64, solverStatus::SolverStatus)::Void
    N = caesarConfig.numParticles

    # Update the status parameters
    solverStatus.isAttached = true
    solverStatus.attachedSessionTimestamp = string(unix2datetime(time()))
    solverStatus.userId = userId
    solverStatus.robotId = robotId
    solverStatus.sessionId = sessionId

    itercount = 0
    while (iterations > 0 || iterations == -1) # loopctrl for future use
      iterations = iterations == -1 ? iterations : iterations-1 # stop at 0 or continue indefinitely if -1

      tic()
      solverStatus.iteration = itercount

      println("===================CONVERT===================")
      solverStatus.currentStep = "Prep_Convert"
      fg = Caesar.initfg(sessionname=sessionId, cloudgraph=cloudGraph)
      updatenewverts!(fg, N=N)
      println()

      println("=============ENSURE INITIALIZED==============")
      solverStatus.currentStep = "Prep_EnsureInitialized"
      ensureAllInitialized!(fg)
      println()

      println("================MULTI-SESSION================")
      solverStatus.currentStep = "Prep_MultiSession"
      rmInstMultisessionPriors!(cloudGraph, session=sessionId, multisessions=String[])
      println()

      println("====================SOLVE====================")
      solverStatus.currentStep = "Init_Solve"
      fg = Caesar.initfg(sessionname=sessionId, cloudgraph=cloudGraph)

      setBackendWorkingSet!(cloudGraph.neo4j.connection, sessionId)

      println("Get local copy of graph")

      solverStatus.currentStep = "Init_LocalGraphCopy"
      if fullLocalGraphCopy!(fg)
        (savejlds && itercount == 0) ? slamindbsavejld(fg, sessionId, itercount) : nothing
        itercount += 1

        println("-------------Ensure Initialization-----------")
        solverStatus.currentStep = "Solve_EnsureInitialized"
        ensureAllInitialized!(fg)

        println("------------Bayes (Junction) Tree------------")
        solverStatus.currentStep = "Solve_BuildBayesTree"
        tree = wipeBuildNewTree!(fg,drawpdf=drawbayestree)

        solverStatus.currentStep = "Solve_InferOverTree"
        if !recursivesolver
          inferOverTree!(fg, tree, N=N)
        else
          inferOverTreeR!(fg, tree, N=N)
        end
        savejlds ? slamindbsavejld(fg, sessionId, itercount) : nothing
      else
        sleep(0.2)
      end
      solverStatus.lastIterationDurationSeconds = toc()
      solverStatus.currentStep = "Idle"
    end

    solverStatus.isAttached = false
    solverStatus.detachedSessionTimestamp = string(unix2datetime(time()))
    solverStatus.userId = ""
    solverStatus.robotId = ""
    solverStatus.sessionId = ""
    return nothing
end

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

  itercount = 0
  while loopctrl[1] && (iterations > 0 || iterations == -1) # loopctrl for future use
    iterations = iterations == -1 ? iterations : iterations-1 # stop at 0 or continue indefinitely if -1
    println("===================CONVERT===================")
    fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)
    updatenewverts!(fg, N=N)
    println()

    println("=============ENSURE INITIALIZED==============")
    ensureAllInitialized!(fg)
    println()

    println("================MULTI-SESSION================")
    rmInstMultisessionPriors!(cloudGraph, session=session, multisessions=addrdict["multisession"])
    println()

    println("====================SOLVE====================")
    fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

    setBackendWorkingSet!(fg.cg.neo4j.connection, session)

    println("get local copy of graph")

    if fullLocalGraphCopy!(fg)
      (savejlds && itercount == 0) ? slamindbsavejld(fg, addrdict["session"], itercount) : nothing
      itercount += 1

      println("-------------Ensure Initialization-----------")
      ensureAllInitialized!(fg)

      println("------------Bayes (Junction) Tree------------")
      tree = wipeBuildNewTree!(fg,drawpdf=drawbayestree)
      if !recursivesolver
        inferOverTree!(fg, tree, N=N)
      else
        inferOverTreeR!(fg, tree, N=N)
      end
      savejlds ? slamindbsavejld(fg, addrdict["session"], itercount) : nothing
    else
      sleep(0.2)
    end
  end
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
