# example of ros service in julia

### Get ROS started

# $ export PYTHON=/usr/bin/python2.7
@assert ENV["PYTHON"]=="/usr/bin/python2.7"
# ]build PyCall

using RobotOS

# standard types
@rosimport geometry_msgs.msg: Point, Pose2D
# bespoke types
@rosimport caesar_ros.srv: AddNode, AddFactor

# generate the types and load
rostypegen()
using .geometry_msgs.msg
using .caesar_ros.srv



### Load Caesar mmisam stuff

using RoME



# yes globals are very bad for performance and memory, but this is a PoC
global fg = initfg()


# service callbacks
function add_node(req::AddNodeRequest)
    global fg

    reply = AddNodeResponse()
    @show req

    # addVariable!(fg, name, softtype)

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

    add_node_srv = Service("AddNode",caesar_ros.srv.AddNode, add_node)

    loop(pub)


end

main()













#
