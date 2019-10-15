"""
    Proof of concept for Caesar-ROS integration
    (check Caesar wiki for details/instructions)
"""

using RobotOS

# standard types
@rosimport geometry_msgs.msg: Point, Pose2D
# bespoke types
@rosimport caesar_ros.srv: AddVariable, AddPrior3D, AddFactorZPR, AddFactorXYH

# generate the types and load
rostypegen()
using .geometry_msgs.msg
using .caesar_ros.srv

### Load Caesar mmisam stuff
using RoME
using DistributedFactorGraphs

using JSON2
using DocStringExtensions

# TODO: avoid having global fg (look into other patterns)
global fg = initfg()

"""
    $SIGNATURES

Service callback for adding a variable to the factor graph.

Notes
- See https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/issues/134
"""
function add_variable(req::AddVariableRequest)
    @debug("received add_variable request", req)
    reply = AddVariableResponse("failed to add variable") 
    type_symbol = Symbol(req.type)
    if in(type_symbol, [:Pose2, :Pose3, :Point3])
        global fg
        vars=ls(fg)
        id = Symbol(req.id)
        if(!in(id,vars))
            retv = addVariable!(fg, id, getfield(Main, type_symbol))
            reply.status = JSON2.write(packVariable(fg,retv))
        else
            @warn("Failed to add variable - repeat variable")
        end
    else
        @warn("Failed to add variable - unsupported type", type_symbol)
    end
    return(reply)
end


"""
    $SIGNATURES

Service callback for adding a binary XYH partial factor.
"""
function add_factor_xyh(req::AddFactorXYHRequest)
    id0=Symbol(req.id0)
    id1=Symbol(req.id1)
    global fg
    vars = ls(fg)
    reply = AddFactorXYHResponse()
    if (in(id0,fg) & in(id1,fg))
        mu_xyh = [req.x, req.y, req.yaw]
        Sigma_xyh = reshape(req.covariance,3,3)
        z_xyh = Pose3Pose3XYYaw(MvNormal(mu_xyh, Sigma_xyh))
        retv = addFactor(fg, [id0, id1], z_xyh)
        reply.status = JSON2.write(packVariable(retv))
    else
        @warn("Failed to add FactorXYH - missing pose")
        reply.status="failed"
    end

    return(reply)
end

"""
    $SIGNATURES

Service callback for adding a unary  partial factor.
"""
function add_factor_zpr(req::AddFactorZPRRequest)
    id=Symbol(req.id)
    global fg
    vars = ls(fg)
    reply = AddFactorZPRResponse()
    if (in(id,fg) )
        mu_zpr = [req.z, req.p, req.r]
        Sigma_zpr = reshape(req.covariance,3,3)
        z_zpr = PriorPose3ZRP(MvNormal(mu_zpr, Sigma_zpr))
        retv = addFactor(fg,  Symbol(req.id1), z_zpr)
        reply.status = JSON2.write(packVariable(retv))
    else
        @warn("Failed to add FactorZPR - missing pose")
        reply.status="failed"
    end

    return(reply)
end

function main()
    init_node("caesar_srv_endpoint")

    # services - variable
    add_variable_srv = Service("AddVariable",caesar_ros.srv.AddVariable, add_variable)
    # services - factors
    add_factor_zpr_srv = Service("AddFactorZPR",caesar_ros.srv.AddFactorZPR, add_factor_zpr )
    add_factor_xyh_srv = Service("AddFactorXYH",caesar_ros.srv.AddFactorXYH, add_factor_xyh )

    # main loop
    @info "services have been set up; entering main loop"
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end
end

main()
