# Debug packed type conversions

# 0. Load required packages
using Caesar, CloudGraphs, RoME




# 1. Get the authentication and session information
include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
user_config["session"] = "TEST_PACKING_CONVERTERS"
addrdict["robotId"] = "TestRobot"




# Start with an empty graph (local dictionary version) # fg = initfg(sessionname="SLAM2D_TUTORIAL")
## TODO -- ISSUE Julia 0.6.0-0.6.2 dives into some StackOverflow problem using the functions, but fine when called separately.
fg = Caesar.initfg(sessionname=user_config["session"], robotname=addrdict["robotId"], cloudgraph=backend_config)


# first pose :x0
addNode!(fg, :x0, Pose2)

# also add a PriorPose2 to pin the first pose at a fixed location
addFactor!(fg, [:x0], PriorPose2(zeros(3,1), 0.01*eye(3), [1.0]))


addNode!(fg, :x1, Pose2)

addFactor!(fg, [:x0;:x1], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )






## Test by solving locally

using IncrementalInference

tree = wipeBuildNewTree!(fg)
inferOverTreeR!(fg, tree)


0
































#
