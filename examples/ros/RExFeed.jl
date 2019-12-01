"""
    Proof of concept for Caesar-ROS integration
    (check Caesar wiki for details/instructions)

    To do:
    - re-enable JSON replies
    - switch to a pattern that does not require use of global variables
    - periodic export of factor graph object

    To run:
    - source devel/setup.bash in all 3 terminals
    - run roscore in one terminal
    - then rosbag play data/lidar_radar.bag
    - and julia RExFeed.jl in third.
"""

### INIT
using RobotOS
using JSON2

# standard types
@rosimport sensor_msgs.msg: PointCloud2
# @rosimport seagrant_msgs: Radar

rostypegen()
using .sensor_msgs.msg

#   ## Load Caesar mmisam stuff
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

# Set this
global max_samples = 100
global dfg_datafolder = "/tmp/rex"

"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleRadar(msg, fg, datastore)
    var_id = Symbol("x",length(ls(fg)))
    # println("Adding variable ", var_id, " on new radar ping")
    variable = addVariable!(fg, var_id, Pose2)

    # Make a big data entry in the graph
    element = GeneralBigDataEntry(fg, variable, :RADAR, mimeType="/radar_pointcloud_0;dataformat=Float32*[[X,Y,Z]]*12")
    # Set it in the store
    addBigData!(datastore, element, msg.data)
    # Add the entry to the graph
    addBigDataEntry!(variable, element)
end

"""
    $SIGNATURES

Message callback for LiDAR point clouds. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleLidar!(msg, fg, datastore)
    var_id = Symbol("x",length(ls(fg)))
    println("Adding variable ", var_id, " on new lidar scan")
    variable = addVariable!(fg, var_id, Pose2)

    # NOTE testing only; we need to either serialize the whole message, or to repack the point cloud using our own schema
    global max_samples = max_samples-1

    # Make a big data entry in the graph
    element = GeneralBigDataEntry(fg, variable, :LIDAR, mimeType="/velodyne_points;dataformat=Float32*[[X,Y,Z]]*32")
    # Set it in the store
    # NOTE: If JSON, then do this to get to Vector{UInt8} - # byteData = Vector{UInt8}(JSON2.write(xyzLidarF32))
    addBigData!(datastore, element, msg.data)
    # Add the entry to the graph
    addBigDataEntry!(variable, element)
end

function main()
    # Not a great pattern but just for proof of concept.
    global max_samples
    global dfg_datafolder

    init_node("rex_feed")

    fg = initfg()
    datastore = FileDataStore("$dfg_datafolder/bigdata")

    radar_sub = Subscriber{sensor_msgs.msg.PointCloud2}("/radar_pointcloud0", handleRadar, (fg,datastore,), queue_size = 10)
    lidar_sub = Subscriber{sensor_msgs.msg.PointCloud2}("/velodyne_points", handleLidar!, (fg,datastore,), queue_size = 10)

    @info "subscribers have been set up; entering main loop"
    loop_rate = Rate(20.0)
    while ! is_shutdown() && (max_samples > 0)
        rossleep(loop_rate)
    end

    @info "Collected max samples, exiting..."
    # After the graph is built, for now we'll save it to drive to share.
    # Save the DFG graph with the following:
    @info "Saving DFG to $dfg_datafolder/dfg"
    saveDFG(fg, "$dfg_datafolder/dfg")

end

main()
