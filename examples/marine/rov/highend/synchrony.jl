#=
LCM Server: an LCM interface to Caesar.jl
=#

using Base.Dates
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
        if !handle(lcm_node)
            break
        end
    end
end

# 0. Constants
println("[Caesar.jl] defining constants.")
robotId = "HROV"
sessionId = "LCM_27"
sessionId = strip(sessionId)

# create a SLAM container object
slam_client = SyncrSLAM(robotId, sessionId, nothing)

# initialize a new session ready for SLAM using the built in SynchronySDK
println("[Caesar.jl] Setting up remote solver")
initialize!(slam_client)

# Set up the robot
setRobotParameters!(slam_client)
robotConfig = Syncr.getRobotConfig(slam_client.syncrconf, slam_client.robotId)

# TODO - should have a function that allows first pose and prior to be set by user.

# create new handlers to pass in additional data
lcm_pose_handler = (channel, message_data) -> handle_poses!(slam_client, message_data )
lcm_odom_handler = (channel, message_data) -> handle_partials!(slam_client, message_data )
lcm_prior_handler = (channel, message_data) -> handle_priors!(slam_client, message_data )
lcm_cloud_handler = (channel, message_data) -> handle_clouds!(slam_client, message_data )
lcm_loop_handler = (channel, message_data) -> handle_loops!(slam_client, message_data )

# create LCM object and subscribe to messages on the following channels
logfile = robotdata("rovlcm_singlesession_01")
lcm_node = LCMLog(logfile) # for direct log file access

# poses
subscribe(lcm_node, "CAESAR_POSES", lcm_pose_handler, pose_node_t)

# factors
subscribe(lcm_node, "CAESAR_PARTIAL_XYH", lcm_odom_handler)
subscribe(lcm_node, "CAESAR_PARTIAL_ZPR", lcm_prior_handler, prior_zpr_t)
# loop closures come in via p3p3nh factors
subscribe(lcm_node, "CAESAR_PARTIAL_XYH_NH", lcm_loop_handler, pose_pose_xyh_nh_t)

# sensor data
subscribe(lcm_node, "CAESAR_POINT_CLOUDS", lcm_cloud_handler, point_cloud_t)

println("[Caesar.jl] Running LCM listener")
listener!(lcm_node)

println(" --- Now we can set all the nodes to ready so the solver picks up on them.")
putReady(slam_client.syncrconf, robotId, sessionId, true)

println(" --- Done! Now we can run the solver on this dataset!")
# TODO: Tell PilotFish to solve.

# Todo - call Sychrony to start the solver

#############################
####### Visualization #######
#############################

# Ref: https://github.com/rdeits/MeshCat.jl/blob/master/demo.ipynb

# NOTE: WIP!
# I'd like the SDK to do this natively...

using MeshCat
using CoordinateTransformations
import GeometryTypes: HyperRectangle, Vec, Point, HomogenousMesh, SignedDistanceField
import ColorTypes: RGBA, RGB

# Create a new visualizer instance
vis = Visualizer()
open(vis)

# Retrieve all variables and render them.
println("Retrieving all variables and rendering them...")
nodes = getNodes(slam_client.syncrconf, robotId, sessionId)
for node in nodes
    println(" - Rendering $node")
    triad = Triad()
    setobject!(vis["Triad $node"], triad)
    settransform!(vis["Triad $node"], Translation(node,node,node))
end
