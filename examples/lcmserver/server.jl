#=
LCM Server: an LCM interface to Caesar.jl

=#
using JLD
using Caesar, RoME
using TransformUtils, Rotations, CoordinateTransformations
using Distributions
using PyCall, PyLCM
using LibBSON

function gen_bindings()
    @show lcmtpath = joinpath(dirname(@__FILE__),"lcmtypes")
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_point_cloud_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_node_t.lcm`)
    run(`lcm-gen -p --ppath $(lcmtpath) $(lcmtpath)/rome_pose_pose_nh_t.lcm`)
    println("Adding lcmtypes dir to Python path: $(lcmtpath)")
    unshift!(PyVector(pyimport("sys")["path"]),lcmtpath)
end

println("[Caesar.jl] (re)generating LCM bindings")
gen_bindings()

println("[Caesar.jl] Importing LCM message types")
@pyimport rome 


function initialize!(backend_config,
                    user_config)
    # TODO: init interface should be made cleaner/more straight forward
    println("[Caesar.jl] Setting up factor graph")
    fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)
    println("[Caesar.jl] Creating SLAM client/object")
    return  SLAMWrapper(fg, nothing, 0)
end

"""

Adds pose nodes to graph with a prior on Z, pitch, and roll.
"""
function handle_odometry!(slam::SLAMWrapper,
                       message_data)

    println("[Caesar.jl] Received message ")
    message = rome.pose_pose_nh_t[:decode](message_data)

    source_id = message[:node_1_id]
    destination_id = message[:node_2_id]

    mean = message[:mean]
    covar = message[:covar]
    t = [mean[1], mean[2], mean[3]]
    qw = mean[4]
    qxyz = [mean[5], mean[6], mean[7]]
    q = Quaternion(qw,qxyz) # why not a (w,x,y,z) constructor?
    pose = SE3(t,q)
    euler = Euler(q)
    
    if source_id == destination_id
        println("[Caesar.jl] First pose")
        # this is the first message, and it does not carry odometry, but the prior on the first node.
        
        # add node
        node_label = Symbol("x1")
        xn = addNode!(slam.fg, node_label, labels=["POSE"]) # this is an incremental inference call
        slam.lastposesym = node_label; # update object

        # add 6dof prior
        initPosePrior = PriorPose3( MvNormal( veeEuler(pose), diagm([covar...]) ) )
        addFactor!(slam.fg, [xn], initPosePrior)

    else
        println("[Caesar.jl] Odometry from $source_id to $destination_id")

        source_label = Symbol("x$source_id") 
        destination_label = Symbol("x$destination_id")

        # get previous node
        xo = getVert(slam.fg, slam.lastposesym)
        odo = pose 

        # add node
        xn = addNode!(slam.fg, destination_label, labels=["POSE"]) # this is an incremental inference call
        slam.lastposesym = destination_label
        
        # add ZPR prior
        println("[Caesar.jl] Adding prior on RPZ")
        rp_dist = MvNormal( [euler.R; euler.P], diagm([covar[6];covar[5]]))
        z_dist = Normal(mean[3], covar[3])
        prior_rpz = PartialPriorRollPitchZ(rp_dist, z_dist) 
        addFactor!(slam.fg, [xn], prior_rpz)

        # add XYH factor
        println("[Caesar.jl] Adding odometry constraint on XYH")
        odo_ea = convert(Euler, odo.R)
        xyh_dist = MvNormal([mean[1];mean[2];odo_ea.Y], diagm([covar[1];covar[2];covar[4]]))
        xyh_factor = PartialPose3XYYaw(xyh_dist)
        addFactor!(slam.fg, [xo;xn], xyh_factor)

        pts = localProduct(slam.fg, slam.lastposesym)
        setVal!(xn, getPoints(pts[1]))
    end
    
end

function handle_factors!(slam::SLAMWrapper,
                         message_data)
    message = rome.pose_pose_nh_t[:decode](message_data)
end

"""
   handle_clouds(slam::SLAMWrapper, message_data) 

Callback for rome_point_cloud_t messages. Adds point cloud to SLAM_Client
"""
function handle_clouds!(slam::SLAMWrapper,
                        message_data)
    # TODO: interface here should be as simple as slam_client.add_pointcloud(pc::SomeCloudType)

    message = rome.point_cloud_t[:decode](message_data)

    id = message[:id]

    last_pose = Symbol("x$(id)")
    println("[Caesar.jl] Got cloud $id")

    # TODO: check if vert exists or not (may happen if messages are lost or out of order)
    vert = getVert(slam.fg, last_pose, api=IncrementalInference.dlapi) # fetch from database
    
    # 2d arrays of points and colors (from LCM data into arrays{arrays})
    points = [[pt[1], pt[2], pt[3]] for pt in message[:points]]
    colors = [[UInt8(c.data[1]),UInt8(c.data[2]),UInt8(c.data[3])] for c in message[:colors]]
    
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
    while true
        handle(lcm_node)
    end
end


# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
# TODO: why keep usrcfg and backendcfg? the former contains the latter
#println("[Caesar.jl] Prompting user for configuration")
@load "usercfg.jld"
backend_config, user_config = standardcloudgraphsetup(addrdict=user_config)

# TODO: need better name for "slam_client"
println("[Caesar.jl] Setting up local solver")
slam_client = initialize!(backend_config,user_config)

# TODO: should take default install values as args?
# TODO: supress/redirect standard server output to log
# NOTE: neo4j: u=neo4j, p=imusing2fa
# NOTE: "Please also enter information for:" 

# create new handlers to pass in additional data 
lcm_pose_handler = (channel, message_data) -> handle_odometry!(slam_client, message_data )
#lcm_factor_handler = (channel, message_data) -> handle_factors!(slam_client, message_data )
lcm_cloud_handler = (channel, message_data) -> handle_clouds!(slam_client, message_data )

# create LCM object and subscribe to messages on the following channels
lcm_node = LCM()
subscribe(lcm_node, "ROME_FACTORS", lcm_pose_handler)
#subscribe(lcm_node, "ROME_FACTORS", lcm_factor_handler)
subscribe(lcm_node, "ROME_POINT_CLOUDS", lcm_cloud_handler)

println("[Caesar.jl] Running LCM listener")
listener!(slam_client, lcm_node)

# TODO: handle termination
# TODO: reply with updates
