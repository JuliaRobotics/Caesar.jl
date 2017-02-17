# convert uploaded graph in DB to MM-iSAM ready form

using Caesar, CloudGraphs


# connect to the server, CloudGraph stuff
dbaddress = length(ARGS) > 0 ? ARGS[1] : "localhost"
dbusr = length(ARGS) > 1 ? ARGS[2] : ""
dbpwd = length(ARGS) > 2 ? ARGS[3] : ""

mongoaddress = length(ARGS) > 3 ? ARGS[4] : "localhost"

session = length(ARGS) > 4 ? string(ARGS[5]) : ""

clearslamindbdata = length(ARGS) > 5 && string(ARGS[6]) == "clear" ? true : false
Nparticles = length(ARGS) > 5 && !clearslamindbdata ? parse(Int,string(ARGS[6])) : 100

println("Attempting to solve session $(session) with $(Nparticles) particles per marginal...")


configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()

if !clearslamindb
  N=100
  fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

  updatenewverts!(fg, N=N)
else
  println("Clearing slamindb data, leaving front-end data, session: $(session)")
  resetentireremotesession(conn,session)
end


println("Done.")
