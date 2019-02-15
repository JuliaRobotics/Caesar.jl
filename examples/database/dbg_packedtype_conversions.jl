# Debug packed type conversions

# 0. Load required packages
using Caesar, Distributions




# 1. Get the authentication and session information
include(joinpath(ENV["HOME"],"Documents","blandauthlocal.jl"))
backend_config, user_config = standardcloudgraphsetup(addrdict=addrdict)
user_config["session"] = "TEST_PACKING_CONVERTERS"
addrdict["robotId"] = "TestRobot"




# Start with an empty graph (local dictionary version) # fg = initfg(sessionname="SLAM2D_TUTORIAL")
fg = Caesar.initfg(sessionname=user_config["session"], robotname=addrdict["robotId"], cloudgraph=backend_config)


# first pose :x0
addVariable!(fg, :x0, Pose2)

# also add a PriorPose2 to pin the first pose at a fixed location
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*eye(3))) )




p1 = getVert(fg, :x0f1, nt=:fnc)
p1.attributes["data"]



addVariable!(fg, :x1, Pose2)
addFactor!(fg, [:x0;:x1], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )


addVariable!(fg, :x2, Pose2)
addFactor!(fg, [:x1;:x2], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )



## Test by solving locally

tree = wipeBuildNewTree!(fg)
inferOverTreeR!(fg, tree)




# newLandmark = VariableRequest("l1", "Point2", nothing, ["LANDMARK"])
# response = addVariable(graffConfig, robotId, sessionId, newLandmark)
# newBearingRangeFactor = BearingRangeRequest("x1", "l1",
#                           DistributionRequest("Normal", Float64[0; 0.1]),
#                           DistributionRequest("Normal", Float64[20; 1.0]))

addVariable!(fg, :l1, Point2, labels=["LANDMARK";])

br = Pose2Point2BearingRange(Normal(0, 0.1), Normal(20, 1.0))

addFactor!(fg, [:x1;:l1], br, autoinit=true )







using RoMEPlotting



drawPoses(fg)

drawPosesLandms(fg)








#
