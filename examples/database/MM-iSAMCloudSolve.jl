# addprocs(7)
# ask first for user convenience
using Caesar

include("/home/dehann/Documents/blandauthlocal.jl")
cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true, addrdict=addrdict)

using RoME
using CloudGraphs, Neo4j

# using IncrementalInference

# session = addrdict["session"]
session = "TestHexagonalDrive11"
addrdict["num particles"] = "100"
Nparticles = parse(Int, addrdict["num particles"])
println("Attempting to solve session $(session) with $(Nparticles) particles per marginal...")


fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

setBackendWorkingSet!(fg.cg.neo4j.connection, session)

# TODO -- incremental graph and subgraphs are works in progress
while true
  println("=================================================")
  fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

  setBackendWorkingSet!(fg.cg.neo4j.connection, session)

  println("get local copy of graph")

  # removeGenericMarginals!(conn) # function should not be necessary, but fixes a minor bug following elimination algorithm
  if fullLocalGraphCopy!(fg)
    tree = wipeBuildNewTree!(fg,drawpdf=true)
    # removeGenericMarginals!(conn)

    # while true # repeat while graph unchanged
      # okay now do the solve
      inferOverTree!(fg, tree, N=Nparticles)
      # if true # current hack till test is inserted
      #   break;
      # end
    # end
  else
    sleep(0.2)
  end
end



# fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)
# fullLocalGraphCopy!(fg, conn)
#
# ls(fg)
#
# tree = wipeBuildNewTree!(fg,drawpdf=true)
#
# inferOverTreeR!(fg, tree, N=100)

  #
