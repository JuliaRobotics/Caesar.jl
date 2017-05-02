#=
LCM Server: an LCM interface to Caesar.jl

=#

using JLD
using Caesar, RoME
using TransformUtils, Rotations, CoordinateTransformations
using Distributions
using PyCall, PyLCM
using LibBSON
using CloudGraphs # for sorryPedro

function gen_bindings()
    @show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/caesar_point_cloud_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/caesar_pose_node_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/caesar_pose_pose_nh_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/caesar_pose_pose_xyh_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/caesar_pose_pose_xyh_nh_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/caesar_prior_zpr_t.lcm`)
    println("Adding lcmtypes dir to Python path: $(lcmtpath)")
    unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
end

println("[Caesar.jl] (re)generating LCM bindings")
gen_bindings()

println("[Caesar.jl] Importing LCM message types")
@pyimport caesar

function initialize!(backend_config,
                    user_config)
    println("[Caesar.jl] Setting up factor graph")
    fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)
    println("[Caesar.jl] Creating SLAM client/object")
    return  SLAMWrapper(fg, nothing, 0)
end

function setRobotParameters!(cg::CloudGraph, session::AbstractString)
  hauvconfig = Dict()
  hauvconfig["robot"] = "hauv"
  hauvconfig["bTc"] = [0.0;0.0;0.0; 1.0; 0.0; 0.0; 0.0]
  hauvconfig["bTc_format"] = "xyzqwqxqyqz"
  # currently unused, but upcoming
  hauvconfig["pointcloud_description_name"] = "BSONpointcloud"
  hauvconfig["pointcloud_color_description_name"] = "BSONcolors"
  # robotdata = json(hauvconfig).data

  # Actually modify the databases
  insertrobotdatafirstpose!(cg, session, hauvconfig)
  nothing
end

"""
Adds pose nodes to graph with a prior on Z, pitch, and roll.
"""
function handle_poses!(slam::SLAMWrapper,
                       message_data)
    message = caesar.pose_node_t[:decode](message_data)
    id = message[:id]
    println("[Caesar.jl] Received pose message for x$(id)")

    mean = message[:mean]
    covar = message[:covar]
    t = [mean[1], mean[2], mean[3]]
    qw = mean[4]
    qxyz = [mean[5], mean[6], mean[7]]
    q = Quaternion(qw,qxyz) # why not a (w,x,y,z) constructor?
    pose = SE3(t,q)
    euler = Euler(q)

    node_label = Symbol("x$(id)")
    xn = addNode!(slam.fg, node_label, labels=["POSE"], dims=6) # this is an incremental inference call
    slam.lastposesym = node_label; # update object

    if id == 0
        println("[Caesar.jl] First pose")
        # this is the first message, and it does not carry odometry, but the prior on the first node.

        # add 6dof prior
        initPosePrior = PriorPose3( MvNormal( veeEuler(pose), diagm([covar...]) ) )
        addFactor!(slam.fg, [xn], initPosePrior)

        # auto init is coming, this code will likely be removed
        initializeNode!(slam.fg, node_label)
    end

    if id == 1
        # set robot parameters in the first pose, this will become a separate node in the future
        # visualization is still assuming parameters in node 1 (not 0), so leaving it here for now
        println("[Caesar.jl] Setting robot parameters")
        setRobotParameters!(slam.fg.cg, slam.fg.sessionname)
    end
end

# handles ZPR priors on poses
function handle_priors!(slam::SLAMWrapper,
                         message_data)


    message = caesar.prior_zpr_t[:decode](message_data)
    id = message[:id]
    println("[Caesar.jl] Adding prior on RPZ to x$(id)")

    node_label = Symbol("x$(id)")
    xn = getVert(slam.fg,node_label)

    z = message[:z]
    pitch = message[:pitch]
    roll = message[:roll]

    var_z = message[:var_z]
    var_pitch = message[:var_pitch]
    var_roll = message[:var_roll]

    rp_dist = MvNormal( [roll;pitch], diagm([var_roll, var_pitch]))
    z_dist = Normal(z, var_z)
    prior_rpz = PartialPriorRollPitchZ(rp_dist, z_dist)
    addFactor!(slam.fg, [xn], prior_rpz)
end


function handle_partials!(slam::SLAMWrapper,
                         message_data)
    message = caesar.pose_pose_xyh_t[:decode](message_data)

    origin_id = message[:node_1_id]
    destination_id = message[:node_2_id]
    origin_label = Symbol("x$(origin_id)")
    destination_label = Symbol("x$(destination_id)")

    println("[Caesar.jl] Adding XYH odometry constraint between(x$(origin_id), x$(destination_id))")

    delta_x = message[:delta_x]
    delta_y = message[:delta_y]
    delta_yaw = message[:delta_yaw]
    
    var_x = message[:var_x]
    var_y = message[:var_y]
    var_yaw = message[:var_yaw]
    
    origin_label, destination_label
    xo = getVert(slam.fg,origin_label)
    xd = getVert(slam.fg,destination_label)

    xyh_dist = MvNormal([delta_x, delta_y, delta_yaw], diagm([var_x, var_y, var_yaw]))
    xyh_factor = PartialPose3XYYaw(xyh_dist)
    addFactor!(slam.fg, [xo;xd], xyh_factor)

    initializeNode!(slam.fg, destination_label)
    println()
end


function handle_loops!(slaml::SLAMWrapper,
                       message_data)
    message = caesar.pose_pose_xyh_nh_t[:decode](message_data)

    origin_id = message[:node_1_id]
    destination_id = message[:node_2_id]
    origin_label = Symbol("x$(origin_id)")
    destination_label = Symbol("x$(destination_id)")

    delta_x = message[:delta_x]
    delta_y = message[:delta_y]
    delta_yaw = message[:delta_yaw]
    
    var_x = message[:var_x]
    var_y = message[:var_y]
    var_yaw = message[:var_yaw]
    confidence = message[:confidence]

    xo = getVert(slaml.fg,origin_label)
    xd = getVert(slaml.fg,destination_label)

    # if (destination_id - origin_id == 1)
    #     warn("Avoiding parallel factor! See: https://github.com/dehann/IncrementalInference.jl/issues/63To ")
    #     return
    # end

    println("[Caesar.jl] Adding XYH-NH loop closure constraint between (x$(origin_id), x$(destination_id))")
    xyh_dist = MvNormal([delta_x, delta_y, delta_yaw], diagm([var_x, var_y, var_yaw]))
    xyh_factor = PartialPose3XYYawNH(xyh_dist ,[1.0-confidence, confidence]) # change to NH
    addFactor!(slaml.fg, [xo;xd], xyh_factor )

    # println("[Caesar.jl] Adding P3P3NH loop closure constraint between (x$(origin_id), x$(destination_id))")

    # # line below fails!
    # lcf = Pose3Pose3NH( MvNormal(veeEuler(rel_pose), diagm(1.0./covar)), [0.5;0.5]) # define 50/50% hypothesis
    # lcf_label = Symbol[origin_label;destination_label]
    
    # addFactor!(slaml.fg, lcf_label, lcf)
end


# type Cloud
#     points::
#     colors::
# end

# function add_pointcloud!(slaml::SLAMWrapper, nodeID::Symbol, cloud::Cloud )
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
   handle_clouds(slam::SLAMWrapper, message_data)

   Callback for caesar_point_cloud_t messages. Adds point cloud to SLAM_Client
"""
function handle_clouds!(slam::SLAMWrapper,
                        message_data)
    # TODO: interface here should be as simple as slam_client.add_pointcloud(nodeID, pc::SomeCloudType)

    # TODO: check for empty clouds!

    message = caesar.point_cloud_t[:decode](message_data)

    id = message[:id]

    last_pose = Symbol("x$(id)")
    println("[Caesar.jl] Got cloud $id")

    # 2d arrays of points and colors (from LCM data into arrays{arrays})
    points = [[pt[1], pt[2], pt[3]] for pt in message[:points]]
    colors = [[UInt8(c.data[1]),UInt8(c.data[2]),UInt8(c.data[3])] for c in message[:colors]]


    # TODO: check if vert exists or not (may happen if messages are lost or out of order)
    vert = getVert(slam.fg, last_pose, api=IncrementalInference.dlapi) # fetch from database

    # push to mongo (using BSON as a quick fix)
    # (for deserialization, see src/DirectorVisService.jl:cachepointclouds!)
    serialized_point_cloud = BSONObject(Dict("pointcloud" => points))
    appendvertbigdata!(slam.fg, vert, "BSONpointcloud", string(serialized_point_cloud).data)
    serialized_colors = BSONObject(Dict("colors" => colors))
    appendvertbigdata!(slam.fg, vert, "BSONcolors", string(serialized_colors).data)
end

# this function handles lcm messages
function listener!(slam::SLAMWrapper,
                   lcm_node::LCMCore.LCM)
    # handle traffic
    # TODO: handle termination
    while true
        handle(lcm_node)
    end
end


#println("[Caesar.jl] Prompting user for configuration")
#@load "usercfg.jld"
#user_config["session"] = "SESSHAUVDEV3"
@load "liljon17.jld"
#  include(joinpath(dirname(@__FILE__),"..","database","blandauthremote.jl"))
# user_config = addrdict
backend_config, user_config = standardcloudgraphsetup(addrdict=user_config)


println("[Caesar.jl] Setting up local solver")
slam_client = initialize!(backend_config,user_config)

# create new handlers to pass in additional data
lcm_pose_handler = (channel, message_data) -> handle_poses!(slam_client, message_data )
lcm_odom_handler = (channel, message_data) -> handle_partials!(slam_client, message_data )
lcm_prior_handler = (channel, message_data) -> handle_priors!(slam_client, message_data )
lcm_cloud_handler = (channel, message_data) -> handle_clouds!(slam_client, message_data )
lcm_loop_handler = (channel, message_data) -> handle_loops!(slam_client, message_data )

# create LCM object and subscribe to messages on the following channels
lcm_node = LCM()
# poses
subscribe(lcm_node, "CAESAR_POSES", lcm_pose_handler)
# factors
subscribe(lcm_node, "CAESAR_PARTIAL_XYH", lcm_odom_handler)
subscribe(lcm_node, "CAESAR_PARTIAL_ZPR", lcm_prior_handler)
subscribe(lcm_node, "CAESAR_PARTIAL_XYH_NH", lcm_loop_handler) # loop closures come in via p3p3nh factors
# sensor data
subscribe(lcm_node, "CAESAR_POINT_CLOUDS", lcm_cloud_handler)

println("[Caesar.jl] Running LCM listener")
listener!(slam_client, lcm_node)

#=
using JLD, Caesar
addrdict = nothing
#to avoid retyping credentials, uncomment the block below

try @load "credentials.jld"
catch addrdict=getcredentials()
@save "credentials.jld" addrdict
end

drawdbdirector(addrdict=addrdict);
=#
