# SLAMinDB service functions



function slamindb(;addrdict=nothing,
            N::Int=-1,
            loopctrl::Vector{Bool}=Bool[true]  )
  #

  nparticles = false
  if N > 0
    addrdict["num particles"] = string(N)
  else
    nparticles = true
  end

  cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict, nparticles=nparticles)

  N = parse(Int, addrdict["num particles"])

  while loopctrl[1] # loopctrl for future use
    println("===================CONVERT===================")
    fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)
    updatenewverts!(fg, N=N)
    println()

    println("====================SOLVE====================")
    fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

    setBackendWorkingSet!(fg.cg.neo4j.connection, session)

    println("get local copy of graph")

    # removeGenericMarginals!(conn) # function should not be necessary, but fixes a minor bug following elimination algorithm
    if fullLocalGraphCopy!(fg)
      tree = wipeBuildNewTree!(fg,drawpdf=true)
      # removeGenericMarginals!(conn)
      inferOverTree!(fg, tree, N=N)
    else
      sleep(0.2)
    end
  end
  nothing
end


function clearConvertedinDB()






















#
