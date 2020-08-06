# tutorial on conventional 2D SLAM example

# 0. Load required packages
using Revise
using RoME, Distributions
using DistributedFactorGraphs


# 1. Setting up our DistributedFactorGraph layers...
# Configure the mirror here.
# include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
# backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
# user_config["session"] = "SLAM2D_TUTORIAL"
# addrdict["robotId"] = "TestRobot"

# Configuring DFG with a local Graphs.jl instance and our mirror.
dfg = DistributedFactorGraph()
include("CloudDFGAPI.jl")
push!(dfg.mirrors, cloudDFG)
info(dfg)

####### ---- MAIN Example

# ... And away we go!
# fg = initfg()

# Add a first pose variable - x0
addV!(dfg, :x0, Pose2()) # , tags=[:POSE]
# Add a PriorPose2 to pin the first pose at a fixed location
addF!(dfg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*eye(3))) )

# ls(fg, :x0)
# isInitialized(fg, :x0)
# # to debug the zero lengths issue
# ensureAllInitialized!(fg)
# getVal(fg, :x0)
# Prior(MvNormal([0.0;0.0;0], diagm([1.0;1.0;0.01].^2)))

# Drive around in a hexagon
for i in 0:5
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  addVariable!(fg, nsym, Pose2) # , tags=[:VARIABLE;:POSE]
  addFactor!(fg, [psym;nsym], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )
  # Pose2Pose2_NEW(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))
en





# Graphs.plot(fg.g)
isInitialized(fg, :x5)
isInitialized(fg, :x6)



ensureAllInitialized!(fg)


using RoMEPlotting

drawPoses(fg)


getVal(fg, :x0)
v = getVert(fg, :x0)
getVal(v)

importall CloudGraphs

exvid = fg.IDs[:x0]
neoID = fg.cgIDs[exvid]
cvr = CloudGraphs.get_vertex(fg.cg, neoID, false)
exv = CloudGraphs.cloudVertex2ExVertex(cvr)
getVal(exv)
# getData(exv)


getData(fg.g.vertices[1])


tree = wipeBuildNewTree!(fg)
# inferOverTree!(fg, tree)
inferOverTreeR!(fg, tree)





using RoMEPlotting, Gadfly





pl = plotKDE(fg, [:x0; :x1; :x2; :x3; :x4; :x5; :x6]);

Gadfly.draw(PDF("tmpX0123456.pdf", 15cm, 15cm), pl)

@async run(`evince tmpX0123456.pdf`)



# pl = drawPoses(fg)
pl = drawPosesLandms(fg)
Gadfly.draw(PDF("tmpPosesFg.pdf", 15cm, 15cm), pl)
@async run(`evince tmpPosesFg.pdf`)



tree = wipeBuildNewTree!(fg)


@async Graphs.plot(tree.bt)


@time inferOverTree!(fg, tree)


# Graphs.plot(tree.bt)

@async Graphs.plot(fg.g)
