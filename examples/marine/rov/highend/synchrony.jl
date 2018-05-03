#=
LCM Server: an LCM interface to Caesar.jl
=#

using Caesar
using Unmarshal
using CaesarLCMTypes
using TransformUtils, Rotations, CoordinateTransformations
using Distributions
using LCMCore
using LibBSON

using RobotTestDatasets

include(joinpath(dirname(@__FILE__), "synchronySDKIntegration.jl"))


"""
Store robot parameters in the centralized database system.
"""
function setRobotParameters!(sslaml::SyncrSLAM)
  rovconf = Dict{String, String}() # TODO relax to Dict{String, Any}
  rovconf["robot"] = "hovering-rov"
  rovconf["bTc"] = "[0.0;0.0;0.0; 1.0; 0.0; 0.0; 0.0]"
  rovconf["bTc_format"] = "xyzqwqxqyqz"
  # currently unused, but upcoming
  rovconf["pointcloud_description_name"] = "BSONpointcloud"
  rovconf["pointcloud_color_description_name"] = "BSONcolors"

  # Actually modify the databases
  Syncr.updateRobotConfig(sslaml.syncrconf, sslaml.robotId, rovconf)

  nothing
end

# Include the LCM handlers...
include(joinpath(dirname(@__FILE__), "lcmHandlers.jl"))

# this function handles lcm msgs
function listener!(lcm_node::Union{LCMCore.LCM, LCMCore.LCMLog})
    # handle traffic
        # TODO: handle termination
    while true
        handle(lcm_node)
    end
end

# 0. Constants
println("[Caesar.jl] defining constants.")
robotId = "HROV"
sessionId = "LCM_27"

# create a SLAM container object
slam_client = SyncrSLAM(robotId, sessionId, nothing)

# initialize a new session ready for SLAM using the built in SynchronySDK
println("[Caesar.jl] Setting up remote solver")
initialize!(slam_client)

# Make sure that this session is not already populated
existingSessions = getSessions(slam_client.syncrconf, sessionId)
if count(session -> session.id == sessionId, existingSessions.sessions) > 0
    error("There is already a session named '$sessionId' for robot '$robotId'. This example will fail if it tries to add duplicate nodes. We strongly recommend providing a new session name.")
end

# Set up the robot
setRobotParameters!(slam_client)
robotConfig = Syncr.getRobotConfig(slam_client.syncrconf, slam_client.robotId)

# TODO - should have a function that allows first pose and prior to be set by user.

# create new handlers to pass in additional data
lcm_pose_handler = (channel, message_data) -> handle_poses!(slam_client, message_data )
lcm_odom_handler = (channel, message_data) -> handle_partials!(slam_client, message_data )
lcm_prior_handler = (channel, message_data) -> handle_priors!(slam_client, message_data )
# lcm_cloud_handler = (channel, message_data) -> handle_clouds!(slam_client, message_data )
lcm_loop_handler = (channel, message_data) -> handle_loops!(slam_client, message_data )

# create LCM object and subscribe to messages on the following channels
logfile = robotdata("rovlcm_singlesession_01")
lcm_node = LCMLog(logfile) # for direct log file access

# poses
subscribe(lcm_node, "CAESAR_POSES", lcm_pose_handler, pose_node_t)

# factors
subscribe(lcm_node, "*CAESAR_PARTIAL_XYH*", lcm_odom_handler, pose_pose_xyh_t)
# subscribe(lcm_node, "CAESAR_PARTIAL_ZPR", lcm_prior_handler, prior_zpr_t)
# loop closures come in via p3p3nh factors
subscribe(lcm_node, "CAESAR_PARTIAL_XYH_NH", lcm_loop_handler, pose_pose_xyh_nh_t)

# sensor data
# subscribe(lcm_node, "CAESAR_POINT_CLOUDS", lcm_cloud_handler, point_cloud_t)

println("[Caesar.jl] Running LCM listener")
listener!(lcm_node)

####################################
########### TESTING ################
####################################

using Unmarshal
packedJson = "{\"vecZij\":[-0.3171846270561218,-7.061707019805908,-0.14232116187023247],\"vecCov\":[0.01,0.0,0.0,0.0,0.01,0.0,0.0,0.0,0.01],\"nullhypothesis\":[0.5,0.5]}"
dictPacked = JSON.parse(packedJson)
packedType = Unmarshal.unmarshal(RoME.PackedPartialPose3XYYawNH, dictPacked)
prior = convert(RoME.PartialPose3XYYawNH, packedType)
####################################
#### TODO stuff
####################################


# type Cloud
#     points::
#     colors::
# end

# function add_pointcloud!(slaml::SyncrSLAM, nodeID::Symbol, cloud::Cloud )
#     # fetch from database
#     vert = getVert(slaml.fg, nodeID, api=IncrementalInference.dlapi)

#     # add points blob
#     serialized_point_cloud = BSONObject(Dict("pointcloud" => cloud.points))
#     appendvertbigdata!(slaml.fg, vert, "BSONpointcloud", string(serialized_point_cloud).data)

#     # add colors blob
#     serialized_colors = BSONObject(Dict("colors" => cloud.colors))
#     appendvertbigdata!(slaml.fg, vert, "BSONcolors", string(serialized_colors).data)
# end

"""
Callback for caesar_point_cloud_t msgs. Adds point cloud to SLAM_Client
"""
function handle_clouds!(slaml::SyncrSLAM,
                        msg::point_cloud_t)
    # TODO: interface here should be as simple as slam_client.add_pointcloud(nodeID, pc::SomeCloudType)

    # TODO: check for empty clouds!

    id = msg.id

    last_pose = Symbol("x$(id)")
    println("[Caesar.jl] Got cloud $id")

    # 2d arrays of points and colors (from LCM data into arrays{arrays})
    points = [[pt[1], pt[2], pt[3]] for pt in msg.points]
    colors = [[UInt8(c.data[1]),UInt8(c.data[2]),UInt8(c.data[3])] for c in msg.colors]


    # TODO: check if vert exists or not (may happen if msgs are lost or out of order)
    vert = getVert(slaml, last_pose, api=IncrementalInference.dlapi) # fetch from database

    # push to mongo (using BSON as a quick fix)
    # (for deserialization, see src/DirectorVisService.jl:cachepointclouds!)
    serialized_point_cloud = BSONObject(Dict("pointcloud" => points))
    appendvertbigdata!(slaml, vert, "BSONpointcloud", string(serialized_point_cloud).data)
    serialized_colors = BSONObject(Dict("colors" => colors))
    appendvertbigdata!(slaml, vert, "BSONcolors", string(serialized_colors).data)
end
