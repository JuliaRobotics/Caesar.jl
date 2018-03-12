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

function slamindb(;addrdict=nothing,
            N::Int=-1,
            loopctrl::Vector{Bool}=Bool[true],
            iterations::Int=-1,
            multisession::Bool=false,
            savejlds::Bool=false,
            recursivesolver::Bool=false,
            drawbayestree::Bool=false  )
  #

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
