addprocs(7)

using Caesar, RoME
using CloudGraphs, Neo4j

# using IncrementalInference

# connect to the server, CloudGraph stuff
dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
dbusr = length(ARGS) > 1 ? ARGS[2] : ""
dbpwd = length(ARGS) > 2 ? ARGS[3] : ""

mongoaddress = length(ARGS) > 3 ? ARGS[4] : "localhost"

session = length(ARGS) > 4 ? string(ARGS[5]) : ""

Nparticles = length(ARGS) > 5 ? parse(Int,string(ARGS[6])) : 100

println("Attempting to solve session $(session) with $(Nparticles) particles per marginal...")


# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))
# session = "SESSROX"


# Connect to database
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()


# TODO -- MAKE INCREMENAL in graph, SUBGRAPHS work in progress!!!!!
while true
  println("=================================================")
  fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

  setBackendWorkingSet!(conn, session)

  println("get local copy of graph")

  # removeGenericMarginals!(conn) # function should not be necessary, but fixes a minor bug following elimination algorithm
  if fullLocalGraphCopy!(fg, conn)
    tree = wipeBuildNewTree!(fg,drawpdf=false)
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
