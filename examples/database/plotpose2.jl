# quick script to draw Pose2D what is in the DB

using Caesar, RoME, Gadfly

# # connect to the server, CloudGraph stuff
dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
println("Taking Neo4j database address as $(dbaddress)...")

dbusr = length(ARGS) > 1 ? ARGS[2] : ""
dbpwd = length(ARGS) > 2 ? ARGS[3] : ""

mongoaddress = length(ARGS) > 3 ? ARGS[4] : "localhost"
println("Taking Mongo database address as $(mongoaddress)...")

session = length(ARGS) > 4 ? string(ARGS[5]) : ""
println("Attempting to draw session $(session)...")


# Connection to database
# register types of interest in CloudGraphs
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()

# fieldnames(fg.cg.neo4j)

fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

# setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fg, conn)

println("Plotting...")
pl = drawPosesLandms(fg)

println("Rendering plotpose2_$(session).pdf...")
draw(PDF("plotpose2_$(session).pdf",40cm,40cm),pl)

#
