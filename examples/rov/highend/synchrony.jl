#=
LCM Server: an LCM interface to Caesar.jl

=#

using Caesar
using LCMCore, CaesarLCMTypes
using TransformUtils, Rotations, CoordinateTransformations, Distributions
using LibBSON, Unmarshal

using RobotTestDatasets

# Does: using SynchronySDK
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


"""
Adds pose nodes to graph with a prior on Z, pitch, and roll.
"""
function handle_poses!(slaml::SyncrSLAM,
                       msg::pose_node_t)
    #
    id = msg.id
    println("[Caesar.jl] Received pose msg for x$(id)")

    mean = msg.mean
    covar = msg.covar
    t = [mean[1], mean[2], mean[3]]
    qw = mean[4]
    qxyz = [mean[5], mean[6], mean[7]]
    q = Quaternion(qw,qxyz) # why not a (w,x,y,z) constructor?
    pose = SE3(t,q)
    euler = Euler(q)

    node_label = Symbol("x$(id)")
    xn = addNode!(slaml, node_label, Pose3) # this is an incremental inference call
    slaml.lastposesym = node_label; # update object

    if id == 1
        println("[Caesar.jl] First pose")
        # this is the first msg, and it does not carry odometry, but the prior on the first node.

        # add 6dof prior
        initPosePrior = PriorPose3( MvNormal( veeEuler(pose), diagm([covar...]) ) )
        addFactor!(slaml, [xn], initPosePrior)

        # auto init is coming, this code will likely be removed
        initializeNode!(slaml, node_label)

        # set robot parameters in the first pose, this will become a separate node in the future
        println("[Caesar.jl] Setting robot parameters")
        setRobotParameters!(slaml)
    end
end


"""
Handle ZPR priors on poses.
"""
function handle_priors!(slam::SyncrSLAM,
                         msg::prior_zpr_t)

    id = msg.id
    println("[Caesar.jl] Adding prior on RPZ to x$(id)")

    node_label = Symbol("x$(id)")
    xn = getVert(slam,node_label)

    z = msg.z
    pitch = msg.pitch
    roll = msg.roll

    var_z = msg.var_z
    var_pitch = msg.var_pitch
    var_roll = msg.var_roll

    rp_dist = MvNormal( [roll;pitch], diagm([var_roll, var_pitch]))
    z_dist = Normal(z, var_z)
    prior_rpz = PartialPriorRollPitchZ(rp_dist, z_dist)
    addFactor!(slam, [xn], prior_rpz)
end

"""
Handle partial x, y, and heading odometry constraints between Pose3 variables.
"""
function handle_partials!(slam::SyncrSLAM,
                         msg::pose_pose_xyh_t)

    origin_id = msg.node_1_id
    destination_id = msg.node_2_id
    origin_label = Symbol("x$(origin_id)")
    destination_label = Symbol("x$(destination_id)")

    println("[Caesar.jl] Adding XYH odometry constraint between(x$(origin_id), x$(destination_id))")

    delta_x = msg.delta_x
    delta_y = msg.delta_y
    delta_yaw = msg.delta_yaw

    var_x = msg.var_x
    var_y = msg.var_y
    var_yaw = msg.var_yaw

    origin_label, destination_label
    xo = getVert(slam,origin_label)
    xd = getVert(slam,destination_label)

    xyh_dist = MvNormal([delta_x, delta_y, delta_yaw], diagm([var_x, var_y, var_yaw]))
    xyh_factor = PartialPose3XYYaw(xyh_dist)
    addFactor!(slam, [xo;xd], xyh_factor)

    initializeNode!(slam, destination_label)
    println()
end

"""
Handle loop closure proposals with chance of being a null hypothesis likelihood.
"""
function handle_loops!(slaml::SyncrSLAM,
                       msg::pose_pose_xyh_nh_t)

    origin_id = msg.node_1_id
    destination_id = msg.node_2_id
    origin_label = Symbol("x$(origin_id)")
    destination_label = Symbol("x$(destination_id)")

    delta_x = msg.delta_x
    delta_y = msg.delta_y
    delta_yaw = msg.delta_yaw

    var_x = msg.var_x
    var_y = msg.var_y
    var_yaw = msg.var_yaw
    confidence = msg.confidence

    xo = getVert(slaml.fg,origin_label)
    xd = getVert(slaml.fg,destination_label)

    # if (destination_id - origin_id == 1)
    #     warn("Avoiding parallel factor! See: https://github.com/dehann/IncrementalInference.jl/issues/63To ")
    #     return
    # end

    println("[Caesar.jl] Adding XYH-NH loop closure constraint between (x$(origin_id), x$(destination_id))")
    xyh_dist = MvNormal([delta_x, delta_y, delta_yaw], diagm([var_x, var_y, var_yaw]))
    xyh_factor = PartialPose3XYYawNH(xyh_dist ,[1.0-confidence, confidence]) # change to NH
    addFactor!(slaml, [xo;xd], xyh_factor )

    # println("[Caesar.jl] Adding P3P3NH loop closure constraint between (x$(origin_id), x$(destination_id))")

    # # line below fails!
    # lcf = Pose3Pose3NH( MvNormal(veeEuler(rel_pose), diagm(1.0./covar)), [0.5;0.5]) # define 50/50% hypothesis
    # lcf_label = Symbol[origin_label;destination_label]

    # addFactor!(slaml, lcf_label, lcf)
end


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
sessionId = "LCM_01"

# create a SLAM container object
slam_client = SyncrSLAM(robotId=robotId,sessionId=sessionId)

# initialize a new session ready for SLAM using the built in SynchronySDK
println("[Caesar.jl] Setting up remote solver")
initialize!(slam_client)

setRobotParameters!(slam_client)
# TEST: Getting robotConfig
@show robotConfig = Syncr.getRobotConfig(slam_client.syncrconf, slam_client.robotId)

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
# lcm_node = LCM() # for UDP Ethernet traffic version

# poses
subscribe(lcm_node, "CAESAR_POSES", lcm_pose_handler, pose_pose_nh_t)
# factors
subscribe(lcm_node, "CAESAR_PARTIAL_XYH", lcm_odom_handler, pose_pose_xyh_t)
subscribe(lcm_node, "CAESAR_PARTIAL_ZPR", lcm_prior_handler, prior_zpr_t)
# loop closures come in via p3p3nh factors
subscribe(lcm_node, "CAESAR_PARTIAL_XYH_NH", lcm_loop_handler, pose_pose_xyh_nh_t)
# sensor data
subscribe(lcm_node, "CAESAR_POINT_CLOUDS", lcm_cloud_handler, point_cloud_t)


println("[Caesar.jl] Running LCM listener")
listener!(lcm_node)
