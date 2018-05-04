"""
Adds pose nodes to graph with a prior on Z, pitch, and roll.
"""
function handle_poses!(slam::SyncrSLAM,
                       msg::pose_node_t)
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
    #
    node_label = Symbol("x$(id)")
    varRequest = VariableRequest(node_label, "Pose3", nothing, ["POSE"])
    resp = addVariable(slam.syncrconf, slam.robotId, slam.sessionId, varRequest)
    # xn = addNode!(slaml, node_label, labels=["POSE"], dims=6) # this is an incremental inference call
    # slaml.lastposesym = node_label; # update object
    #
    if id == 0
        println("[Caesar.jl] First pose")
        # this is the first msg, and it does not carry odometry, but the prior on the first node.

        # add 6dof prior
        initPosePrior = RoME.PriorPose3( MvNormal( veeEuler(pose), diagm([covar...]) ) )
        packedPrior = convert(RoME.PackedPriorPose3, initPosePrior)

        # 3. Build the factor request (again, we can make this way easier and transparent once it's stable)
        fctBody = FactorBody(string(typeof(initPosePrior)), string(typeof(packedPrior)), "JSON", JSON.json(packedPrior))
        fctRequest = FactorRequest([node_label], fctBody, false, false)
        @show resp = addFactor(slam.syncrconf, slam.robotId, slam.sessionId, fctRequest)

        # auto init is coming, this code will likely be removed
        # initializeNode!(slaml, node_label)

        # set robot parameters in the first pose, this will become a separate node in the future
        println("[Caesar.jl] Setting robot parameters")
        setRobotParameters!(slam)
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
    # xn = getVert(slam,node_label)

    # 1. Build up the prior
    z = msg.z
    pitch = msg.pitch
    roll = msg.roll
    var_z = msg.var_z
    var_pitch = msg.var_pitch
    var_roll = msg.var_roll
    rp_dist = MvNormal( [roll;pitch], diagm([var_roll, var_pitch]))
    z_dist = Normal(z, var_z)
    prior_rpz = RoME.PartialPriorRollPitchZ(rp_dist, z_dist)

    # 2. Pack the prior (we can automate this step soon, but for now it's hand cranking)
    packed_prior_rpz = convert(RoME.PackedPartialPriorRollPitchZ, prior_rpz)

    # 3. Build the factor request (again, we can make this way easier and transparent once it's stable)
    # fctBody = FactorBody(string(typeof(prior_rpz)), string(typeof(packed_prior_rpz)), "JSON", JSON.json(packed_prior_rpz))
    # fctRequest = FactorRequest([node_label], fctBody, false, false)
    # @show resp = addFactor(slam.syncrconf, slam.robotId, slam.sessionId, fctRequest)
end

"""
Handle partial x, y, and heading odometry constraints between Pose3 variables.
"""
function handle_partials!(slam::SyncrSLAM,
                         msg::Any)
    println(" --- Handling odometry change...")
    return

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
    # @show xo = getVert(slam,origin_label)
    # @show xd = getVert(slam,destination_label)

    xyh_dist = MvNormal([delta_x, delta_y, delta_yaw], diagm([var_x, var_y, var_yaw]))
    xyh_factor = PartialPose3XYYaw(xyh_dist)
    # 2. Pack the prior (we can automate this step soon, but for now it's hand cranking)
    packed_xyh_factor = convert(RoME.PackedPartialPose3XYYaw, xyh_factor)

    # 3. Build the factor request (again, we can make this way easier and transparent once it's stable)
    fctBody = FactorBody(string(typeof(xyh_factor)), string(typeof(packed_xyh_factor)), "JSON", JSON.json(packed_xyh_factor))
    fctRequest = FactorRequest([origin_label; destination_label], fctBody, false, false)
    @show resp = addFactor(slam.syncrconf, slam.robotId, slam.sessionId, fctRequest)

    # addFactor!(slam, [xo;xd], xyh_factor)
    #
    # initializeNode!(slam, destination_label)
    # println()
end

"""
Handle loop closure proposals with chance of being a null hypothesis likelihood.
"""
function handle_loops!(slam::SyncrSLAM,
                       msg::pose_pose_xyh_nh_t)

    println(" --- Handling loop...")
    # return nothing

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

    # @show xo = getVert(slam.fg,origin_label)
    # @show xd = getVert(slam.fg,destination_label)

    # if (destination_id - origin_id == 1)
    #     warn("Avoiding parallel factor! See: https://github.com/dehann/IncrementalInference.jl/issues/63To ")
    #     return
    # end

    println("[Caesar.jl] Adding XYH-NH loop closure constraint between (x$(origin_id), x$(destination_id))")
    xyh_dist = MvNormal([delta_x, delta_y, delta_yaw], diagm([var_x, var_y, var_yaw]))
    xyh_factor = RoME.PartialPose3XYYawNH(xyh_dist ,[1.0-confidence, confidence]) # change to NH
    # 2. Pack the prior (we can automate this step soon, but for now it's hand cranking)
    packed_xyh_factor = convert(RoME.PackedPartialPose3XYYawNH, xyh_factor)

    # 3. Build the factor request (again, we can make this way easier and transparent once it's stable)
    fctBody = FactorBody(string(typeof(xyh_factor)), string(typeof(packed_xyh_factor)), "JSON", JSON.json(packed_xyh_factor))
    fctRequest = FactorRequest([origin_label; destination_label], fctBody, false, false)
    @show resp = addFactor(slam.syncrconf, slam.robotId, slam.sessionId, fctRequest)

    # addFactor!(slaml, [xo;xd], xyh_factor )

    # NOT USED
    # println("[Caesar.jl] Adding P3P3NH loop closure constraint between (x$(origin_id), x$(destination_id))")

    # # line below fails!
    # lcf = Pose3Pose3NH( MvNormal(veeEuler(rel_pose), diagm(1.0./covar)), [0.5;0.5]) # define 50/50% hypothesis
    # lcf_label = Symbol[origin_label;destination_label]

    # addFactor!(slaml, lcf_label, lcf)
end

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
    return

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
