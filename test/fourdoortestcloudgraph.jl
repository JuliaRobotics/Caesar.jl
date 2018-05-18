# a basic create robot type node example

using Caesar, IncrementalInference
using CloudGraphs

using KernelDensityEstimate

# switch IncrementalInference to use CloudGraphs (Neo4j) data layer
# dbaddress = "localhost"
# dbusr = ""
# dbpwd = ""
# mongoaddress = "localhost"
session = "SESSTEST"
include(joinpath(dirname(@__FILE__) ,"blandauth.jl"))
Nparticles = 200
println("Attempting to solve session $(session)...")

configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, dbusr, dbpwd, mongoaddress, 27017, false, "", "");
cloudGraph = connect(configuration, encodePackedType, getpackedtype, decodePackedType);
# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
Caesar.usecloudgraphsdatalayer!()

# Uncomment out for command line operation
# cloudGraph, addrdict = standardcloudgraphsetup(drawdepth=true)
# session = addrdict["session"]
# Nparticles = addrdict["num particles"]



# this is being replaced by cloudGraph, added here for development period
fg = Caesar.initfg(sessionname=session, cloudgraph=cloudGraph)


# Robot navigation and inference type stuff
N=Nparticles
doors = [-100.0;0.0;100.0;300.0]'
cov = [3.0]

# robot style, add first pose vertex
v1 = addNode!(fg,:x1,doors,N=N,labels=["POSE"])

# add a prior for the initial position of the robot
f0  = addFactor!(fg,[v1], Obsv2(doors, cov', [1.0]))

# add second pose vertex
tem = 2.0*randn(1,N)+getVal(v1)+50.0
v2 = addNode!(fg, :x2, tem, N=N,labels=["POSE"])

# now add the odometry factor between them
f1 = addFactor!(fg,[v1;v2],Odo([50.0]',[2.0]',[1.0]))


v3=addNode!(fg,:x3,4.0*randn(1,N)+getVal(v2)+50.0, N=N,labels=["POSE"])
addFactor!(fg,[v2;v3],Odo([50.0]',[4.0]',[1.0]))
f2 = addFactor!(fg,[v3], Obsv2(doors, cov', [1.0]))

v4=addNode!(fg,:x4,2.0*randn(1,N)+getVal(v3)+50.0, N=N,labels=["POSE"])
addFactor!(fg,[v3;v4],Odo([50.0]',[2.0]',[1.0]))

# if true
l1=addNode!(fg, :l1, 0.5*randn(1,N)+getVal(v3)+64.0, N=N,labels=["LANDMARK"])
addFactor!(fg, [v3,l1], Ranged([64.0],[0.5],[1.0]))
addFactor!(fg, [v4,l1], Ranged([16.0],[0.5],[1.0]))
# end


v5=addNode!(fg,:x5,2.0*randn(1,N)+getVal(v4)+50.0, N=N,labels=["POSE"])
addFactor!(fg,[v4;v5],Odo([50.0]',[2.0]',[1.0]))


v6=addNode!(fg,:x6,1.25*randn(1,N)+getVal(v5)+40.0, N=N,labels=["POSE"])
addFactor!(fg,[v5;v6],Odo([40.0]',[1.25]',[1.0]))


v7=addNode!(fg,:x7,2.0*randn(1,N)+getVal(v6) +60.0, N=N,labels=["POSE"])
addFactor!(fg,[v6;v7],Odo([60.0]',[2.0]',[1.0]))

f3 = addFactor!(fg,[v7], Obsv2(doors, cov', [1.0]))


# Now operate with the data in the DB

tree = wipeBuildNewTree!(fg,drawpdf=true)
# run(`evince bt.pdf`)

# gc()
# recursive solving (single process, easy stack trace for debugging)
println("Starting inference over tree")
inferOverTreeR!(fg, tree)


# Get neighbors
# neighs = CloudGraphs.get_neighbors(fg.cg, cv1r)

gt = Dict{Symbol, Array{Float64,2}}()
gt[:x1]=([0.0;1.97304 ]')' # -0.0342366
gt[:x2]=([50.0; 2.83153 ]')' # 49.8797
gt[:x3]=([100.0; 1.65557 ]')' # 99.8351
gt[:x4]=([150.0; 1.64945 ]')' # 148.637
gt[:x5]=([200.0; 1.77992 ]')' # 198.62
gt[:x6]=([240.0; 2.20466 ]')' # 238.492
gt[:x7]=([300.0; 2.14353 ]')' # 298.467
gt[:l1]=([165.0; 1.17284 ]')' # 164.102



# get vertex back from DB
x1neoID = fg.cgIDs[fg.IDs[:x1]]
cv1r = CloudGraphs.get_vertex(fg.cg, x1neoID, false)
v1r = cloudVertex2ExVertex(cv1r)
plotKDE(getKDE(v1r))

plotKDE( getVertKDE(fg, :x2) )



# draw all beliefs
using Gadfly

DOYTICKS = false
xx,ll = ls(fg)
msgPlots = drawHorBeliefsList(fg, xx, gt=gt,nhor=2);
evalstr = ""
for i in 1:length(msgPlots)
    evalstr = string(evalstr, ",msgPlots[$(i)]")
end
pl = eval(parse(string("vstack(",evalstr[2:end],")")));
Gadfly.draw(PDF("cg4doors.pdf",15cm,20cm),pl) # can also do PNG
