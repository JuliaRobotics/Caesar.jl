using IncrementalInference, CloudGraphs, Neo4j

# switch IncrementalInference to use CloudGraphs (Neo4j) data layer

# connect to the server, CloudGraph stuff
dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
dbusr = length(ARGS) > 1 ? ARGS[2] : ""
dbpwd = length(ARGS) > 2 ? ARGS[3] : ""

mongoaddress = length(ARGS) > 3 ? ARGS[4] : "localhost"

session = length(ARGS) > 4 ? utf8(ARGS[5]) : ""
println("Attempting to solve session $(session)...")


configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
IncrementalInference.setdatalayerAPI!()

# Connect to database
conn = cloudGraph.neo4j.connection


# cv2.imwrite('test.png', depth_im)


# TODO -- MAKE INCREMENAL in graph, SUBGRAPHS work in progress!!!!!
while true
  # this is being replaced by cloudGraph, added here for development period
  println("=================================================")
  fg = emptyFactorGraph()
  fg.cg = cloudGraph
  fg.sessionname = session

    setBackendWorkingSet!(conn, session)

    println("get local copy of graph")
    # removeGenericMarginals!(conn) # function should not be necessary, but fixes a minor bug following elimination algorithm
    if fullLocalGraphCopy!(fg, conn)
      tree = wipeBuildNewTree!(fg,drawpdf=false)
      # removeGenericMarginals!(conn)

      # while true # repeat while graph unchanged
        # okay now do the solve
        inferOverTree!(fg, tree, N=100)
        # if true # current hack till test is inserted
        #   break;
        # end
      # end
    else
      sleep(0.2)
    end
end




  #
