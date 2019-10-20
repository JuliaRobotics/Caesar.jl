"""
    Proof of concept for Caesar-ROS integration
    (check Caesar wiki for details/instructions)

    To do:
    - re-enable JSON replies
    - switch to a pattern that does not require use of global variables
    - periodic export of factor graph object
"""

using RobotOS

# standard types
@rosimport geometry_msgs.msg: Point, Pose2D
# bespoke types
@rosimport caesar_ros.srv: AddVariable, AddPrior3D, AddFactorZPR, AddFactorXYH

rostypegen()
using .geometry_msgs.msg
using .caesar_ros.srv

### Load Caesar mmisam stuff
using RoME
using DistributedFactorGraphs

using JSON2
using DocStringExtensions

global fg

"""
    $SIGNATURES

Service callback for adding a variable to the factor graph.

Notes
- See https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/issues/134
"""
function add_variable(req::AddVariableRequest)
    @info("received add_variable request", req)
    global fg
    reply = AddVariableResponse("failed to add variable")
    type_symbol = Symbol(req.type)
    if in(type_symbol, [:Pose2, :Pose3, :Point3])
        global fg
        vars = ls(fg)
        id = Symbol(req.id)
        if(!in(id, vars))
            retv = addVariable!(fg, id, getfield(Main, type_symbol))
            reply.status = JSON2.write(packVariable(fg, retv))
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
function add_factor_xyh(req::AddFactorXYHRequest )
    @info("received add_factor_xyh request", req)
    @show id0=Symbol(req.id0)
    @show id1=Symbol(req.id1)
    global fg
    @show vars = ls(fg)
    @show reply = AddFactorXYHResponse()
    if (in(id0, vars) & in(id1, vars))
        mu_xy = [req.x, req.y]
        Sigma = reshape(req.covariance,3,3)
        Sigma_xy = Sigma[1:2,1:2]
        z_xyh = Pose3Pose3XYYaw(MvNormal(mu_xy, Sigma[1:2,1:2]), Normal(req.yaw, Sigma[3,3]))
        retv = addFactor!(fg, [id0, id1], z_xyh)
        # @show reply.status = JSON2.write(packVariable(fg, retv))
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
function add_factor_zpr(req::AddFactorZPRRequest )
    @info("received add_factor_zpr request", req)
    id = Symbol(req.id)
    global fg
    vars = ls(fg)
    reply = AddFactorZPRResponse()
    if (in(id,vars) )
        mu_z = req.z
        mu_rp = [req.roll, req.pitch]
        Sigma_zpr = reshape(req.covariance, 3, 3)
        sigma_z = Sigma_zpr[1,1]
        Sigma_rp = Sigma_zpr[3:-1:2,3:-1:2]
        z_zpr = PriorPose3ZRP(Normal(mu_z, sigma_z), MvNormal(mu_rp, Sigma_rp))
        retv = addFactor!(fg, [id], z_zpr)
        # reply.status = JSON2.write(packVariable(fg, retv)) # breaks
    else
        @warn("Failed to add FactorZPR - missing pose")
        reply.status="failed"
    end

    return(reply)
end

function main()
    init_node("caesar_srv_endpoint")

    global fg
    fg = initfg()

    # services - variable
    add_variable_srv = Service("AddVariable", caesar_ros.srv.AddVariable, add_variable)
    # services - factors
    add_factor_zpr_srv = Service("AddFactorZPR",caesar_ros.srv.AddFactorZPR, add_factor_zpr)
    add_factor_xyh_srv = Service("AddFactorXYH",caesar_ros.srv.AddFactorXYH, add_factor_xyh )

    # main loop
    @info "services have been set up; entering main loop"
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end
end

main()
