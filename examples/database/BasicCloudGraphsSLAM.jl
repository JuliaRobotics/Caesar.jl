# a basic create robot type node example

using Caesar, RoME
using CloudGraphs
using IncrementalInference


include(joinpath(dirname(@__FILE__) ,"blandauthremote.jl"))

# Connection to database
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration);
conn = cloudGraph.neo4j.connection
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()


session = "TESTPOSE2"

fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)
println("Working with session $(fg.sessionname)...")



N = 100
initCov = diagm([0.03;0.03;0.001].^2)
odoCov = diagm([3.0;3.0;0.01].^2)


# Some starting position
v1 = addNode!(fg, :x1, zeros(3,1), diagm([1.0;1.0;0.1]), N=N,labels=["POSE"])
initPosePrior = PriorPose2(zeros(3,1), initCov, [1.0])
f1  = addFactor!(fg,[v1], initPosePrior)


ppc = Pose2Pose2(([50.0;0.0;pi/2]')', odoCov, [1.0])
v2, f2 = addOdoFG!( fg, ppc ,labels=["POSE"] )

v3, f3 = addOdoFG!( fg, ppc ,labels=["POSE"] )


# Some basic graph operations using the database
drawPoses(fg);

# get vertex back from DB
x1neoID = fg.cgIDs[fg.IDs[:x1]]
cv1r = CloudGraphs.get_vertex(fg.cg, x1neoID, false)
# Get neighbors
neighs = CloudGraphs.get_neighbors(fg.cg, cv1r)



# now solve using separate variable
fgs = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)

setBackendWorkingSet!(conn, session)
fullLocalGraphCopy!(fgs, conn)
tree = wipeBuildNewTree!(fgs)


inferOverTreeR!(fgs, tree)




@show fgs




# # Robot navigation and inference type stuff
# N=100
# doors = [-100.0;0.0;100.0;300.0]'
# cov = [3.0]
#
# # robot style, add first pose vertex
# v1 = addNode!(fg,:x1,doors,N=N)
#
# # add a prior for the initial position of the robot
# f0  = addFactor!(fg,[v1], Obsv2(doors, cov', [1.0]))
#
# # add second pose vertex
# tem = 2.0*randn(1,N)+getVal(v1)+50.0
# v2 = addNode!(fg, :x2, tem, N=N)
#
# # now add the odometry factor between them
# f1 = addFactor!(fg,[v1;v2],Odo([50.0]',[2.0]',[1.0]))






























#
