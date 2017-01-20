# a basic create robot type node example

using Caesar, RoME, CloudGraphs
using IncrementalInference


session = "SESSTEST"
include(joinpath(dirname(@__FILE__) ,"../../test/blandauth.jl"))
println("Attempting to solve session $(session)...")

configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
# Connection to database
conn = cloudGraph.neo4j.connection

# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)

Caesar.usecloudgraphsdatalayer!()

fg = initfg(sessionname=session)
fg.cg = cloudGraph

# Robot navigation and inference type stuff
N=100
doors = [-100.0;0.0;100.0;300.0]'
cov = [3.0]

# robot style, add first pose vertex
v1 = addNode!(fg,:x1,doors,N=N)

# add a prior for the initial position of the robot
f0  = addFactor!(fg,[v1], Obsv2(doors, cov', [1.0]))

# add second pose vertex
tem = 2.0*randn(1,N)+getVal(v1)+50.0
v2 = addNode!(fg, :x2, tem, N=N)

# now add the odometry factor between them
f1 = addFactor!(fg,[v1;v2],Odo([50.0]',[2.0]',[1.0]))

# get vertex back from DB
x1neoID = fg.cgIDs[fg.IDs[:x1]]
cv1r = CloudGraphs.get_vertex(fg.cg, x1neoID, false)

# Get neighbors
neighs = CloudGraphs.get_neighbors(fg.cg, cv1r)































#
