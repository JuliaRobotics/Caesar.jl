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
@rosimport sensor_msgs.msg: PointCloud2

rostypegen()
using .sensor_msgs.msg

### Load Caesar mmisam stuff
using RoME
using DistributedFactorGraphs

using JSON2
using DocStringExtensions

# /gps/fix              10255 msgs    : sensor_msgs/NavSatFix
# /gps/nmea_sentence    51275 msgs    : nmea_msgs/Sentence
# /radar_0               9104 msgs    : seagrant_msgs/radar
# /radar_pointcloud_0    9104 msgs    : sensor_msgs/PointCloud2
# /velodyne_points      20518 msgs    : sensor_msgs/PointCloud2

# function handleGPS(msg, fg)
# end



"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleRadar(msg, fg)
    var_id = Symbol("x",length(ls(fg)))
    # println("Adding variable ", var_id, " on new radar ping")
    addVariable!(fg, var_id, Pose2)
end

"""
    $SIGNATURES

Message callback for LiDAR point clouds. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleLidar!(msg, fg)
    var_id = Symbol("x",length(ls(fg)))
    println("Adding variable ", var_id, " on new lidar scan")
    addVariable!(fg, var_id, Pose2)
    # NOTE testing only; we need to either serialize the whole message, or to repack the point cloud using our own schema

    appendvertbigdata!(fg, var_id, "ROS/sensor_msgs/PointCloud2", (msg))
end


function main()
    init_node("rex_feed")

    fg = initfg()

    radar_sub = Subscriber{PointCloud2}("/radar_pointcloud0", handleRadar, (fg,), queue_size = 10)
    lidar_sub = Subscriber{PointCloud2}("/velodyne_points", handleLidar!, (fg,), queue_size = 10)

    @info "subscribers have been set up; entering main loop"
    loop_rate = Rate(20.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end

end

main()
