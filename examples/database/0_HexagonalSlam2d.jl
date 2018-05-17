# tutorial on conventional 2D SLAM example

# addprocs(3)

# This tutorial shows how to use some of the commonly used factor types
# This tutorial follows from the ContinuousScalar example from IncrementalInference

# @everywhere begin
# using RoME
  # using IncrementalInference, Distributions
  # using TransformUtils
# end # everywhere


using Caesar, RoME
# function initialize!(backend_config,
#                     user_config)
#     println("[Caesar.jl] Setting up factor graph")
#     fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)
#     println("[Caesar.jl] Creating SLAM client/object")
#     return  SLAMWrapper(fg, nothing, 0)
# end

# include("$(ENV["HOME"])/Documents/blandauth.jl")
include("$(ENV["HOME"])/Documents/blandauthlocal.jl"); user_config = addrdict
backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
user_config["session"] = "SESSSLAM2D_TUTORIAL"





# Start with an empty graph (local dictionary version) # fg = initfg(sessionname="SLAM2D_TUTORIAL")
## TODO -- ISSUE Julia 0.6.0-0.6.2 dives into some StackOverflow problem using the functions, but fine when called separately.
fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)

# fg = RoME.initfg(sessionname=addrdict["session"])
# fg = IncrementalInference.emptyFactorGraph()
# user_config["session"]
# fg.sessionname
# fg.sessionname = user_config["session"]
# fg.cg = backend_config

# also add a PriorPose2 to pin the first pose at a fixed location
addNode!(fg, :x0, Pose2, labels=["VARIABLE";"POSE"])
addFactor!(fg, [:x0], PriorPose2(zeros(3,1), 0.01*eye(3), [1.0]))

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
  addNode!(fg, nsym, Pose2, labels=["VARIABLE";"POSE"])
  addFactor!(fg, [psym;nsym], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )
  # Pose2Pose2_NEW(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))
end





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
