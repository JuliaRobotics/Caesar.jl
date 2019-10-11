# example of ros service in julia

### Get ROS started

# $ export PYTHON=/usr/bin/python2.7
@assert ENV["PYTHON"]=="/usr/bin/python2.7"
# ]build PyCall

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
using DocStringExtensions


# yes globals are very bad for performance and memory, but this is a PoC
global fg = initfg()

"""
    $SIGNATURES

Service callback for adding a variable to the factor graph.

Notes
- See https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/issues/134
"""
function add_variable(req::AddVariableRequest)
    global fg

    reply = AddVariableResponse("failed to add variable; incorrect type-- must be Pose2 Pose3")  # reply.status

    sts = Symbol(req.type)
    # if wrong, fail gracefully
    sts == :Pose2 || sts == :Pose3 ? nothing : (return reply)

    softtype = getfield(Main, sts)
    ret = addVariable!(fg, Symbol(req.id), softtype)
    reply.status = string(thing)

    # not sure how to get serialized verions
    return reply
end


function add_factor_xyh(req::AddFactorXYHRequest)
    reply = AddFactorXYHResponse()
    return reply
end

function add_factor_zpr(req::AddFactorZPRRequest)
    reply = AddFactorZPRResponse()
    return reply
end

function add_factor(req::AddFactorRequest)
    global fg

    reply = AddFactorResponse()
    # addFactor!(fg, [from; to], factor)
    return reply
end

function callback(msg::Pose2D, pub_obj::Publisher{Point})
    pt_msg = Point(msg.x, msg.y, 0.0)
    publish(pub_obj, pt_msg)
end

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        npt = Point(rand(), rand(), 0.0)
        publish(pub_obj, npt)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_example")
    pub = Publisher{Point}("pts", queue_size=10)
    sub = Subscriber{Pose2D}("pose", callback, (pub,), queue_size=10)

    add_variable_srv = Service("AddVariable",caesar_ros.srv.AddVariable, add_variable)

    add_factor_zpr_srv = Service("AddFactorZPR",caesar_ros.srv.AddFactorZPR, add_factor_zpr )
    add_factor_xyh_srv = Service("AddFactorXYH",caesar_ros.srv.AddFactorXYH, add_factor_xyh )

    loop(pub)


end

main()













#
