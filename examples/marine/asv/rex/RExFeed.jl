"""
    Proof of concept for Caesar-ROS integration
    (check Caesar wiki for details/instructions)

    To do:
    - re-enable JSON replies
        s- periodic export of factor graph object

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
@rosimport sensor_msgs.msg: NavSatFix
# @rosimport nmea_msgs.msg: Sentence
# seagrant type
# OMG, rosmsg list is awesome by the way...
# Also rosnode info
@rosimport seagrant_msgs.msg: radar

rostypegen()
# No using needed because we're specifying by full name.
# using .sensor_msgs.msg
# using .seagrant_msgs.msg

#   ## Load Caesar mmisam stuff
using RoME
using DistributedFactorGraphs

using JSON2
using DistributedFactorGraphs.DocStringExtensions
using Dates

# /gps/fix              10255 msgs    : sensor_msgs/NavSatFix
# /gps/nmea_sentence    51275 msgs    : nmea_msgs/Sentence
# /radar_0               9104 msgs    : seagrant_msgs/radar
# /radar_pointcloud_0    9104 msgs    : sensor_msgs/PointCloud2
# /velodyne_points      20518 msgs    : sensor_msgs/PointCloud2

# function handleGPS(msg, fg)
# end



"""
    $TYPEDEF
Quick placeholder for the system state - we're going to use timestamps to align all the data.
"""
mutable struct SystemState
    curtimestamp::Float64
    cur_variable::Union{Nothing, DFGVariable}
    var_index::Int
    lidar_scan_index::Int
    max_lidar::Int
    SystemState() = new(-1000, nothing, 0, 0, 3)
end

"""
    $(SIGNATURES)
Update the system state variable if the timestamp has changed (increment variable)
"""
function updateVariableIfNeeded(fg::AbstractDFG, systemstate::SystemState, newtimestamp::Float64)
    # Make a new variable if so.
    if systemstate.curtimestamp == -1000 || systemstate.cur_variable === nothing || systemstate.curtimestamp < newtimestamp
        systemstate.curtimestamp = newtimestamp
        systemstate.cur_variable = addVariable!(fg, Symbol("x$(systemstate.var_index)"), Pose2, timestamp = unix2datetime(newtimestamp))
        systemstate.var_index += 1
        systemstate.lidar_scan_index = 0
    end
    return nothing
end

"""
    $SIGNATURES

Message callback for /radar_0.
"""
function handleRadar!(msg::seagrant_msgs.msg.radar, fg::AbstractDFG, systemstate::SystemState)
    @info "handleRadar" maxlog=10

    # if systemstate.cur_variable === nothing
    #     return nothing
    # end
    # Update the variable if needed
    timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
    updateVariableIfNeeded(fg, systemstate, timestamp)
    @info "[$timestamp] RADAR sample on $(systemstate.cur_variable.label)"

    # Make a data entry in the graph - use JSON2 to just write this (really really verbosely)
    ade,adb = addData!(fg, :radar, systemstate.cur_variable.label, :RADAR, Vector{UInt8}(JSON2.write(msg)))
   
end

"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleRadarPointcloud!(msg::sensor_msgs.msg.PointCloud2, fg::AbstractDFG, systemstate::SystemState)
    @info "handleRadarPointcloud" maxlog=10

    if systemstate.cur_variable === nothing
        # Keyed by the radar, skip if we don't have a variable yet.
        return nothing
    end
    timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
    @info "[$timestamp] RADAR pointcloud sample on $(systemstate.cur_variable.label)"

    # Make a data entry in the graph
    ade,adb = addData!(fg, :radar, systemstate.cur_variable.label, :RADARPC, Vector{UInt8}(JSON2.write(msg)),  mimeType="/radar_pointcloud_0;dataformat=Float32*[[X,Y,Z]]*12")

end

"""
    $SIGNATURES

Message callback for LIDAR point clouds. Adds a variable to the factor graph and appends the scan as a bigdata element.
Note that we're just appending all the LIDAR scans to the variables because we are keying by RADAR.
"""
function handleLidar!(msg::sensor_msgs.msg.PointCloud2, fg::AbstractDFG, systemstate::SystemState)
    @info "handleLidar" maxlog=10
    # Compare systemstate and add the LIDAR scans if we want to.
    if systemstate.cur_variable === nothing
        return nothing
    end
    timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
    @info "[$timestamp] LIDAR pointcloud sample on $(systemstate.cur_variable.label) (sample $(systemstate.lidar_scan_index+1))"

    # Check if we have enough LIDAR's for this variable
    if systemstate.lidar_scan_index >= systemstate.max_lidar
        @warn "Ditching LIDAR sample for this variable, already have enough..."
        return nothing
    end

    # Make a data entry in the graph
    ade,adb = addData!(fg, :lidar, systemstate.cur_variable.label, Symbol("LIDAR$(systemstate.lidar_scan_index)"), Vector{UInt8}(JSON2.write(msg)), mimeType="/velodyne_points;dataformat=Float32*[[X,Y,Z]]*32")

    # NOTE: If JSON, then do this to get to Vector{UInt8} - # byteData = Vector{UInt8}(JSON2.write(xyzLidarF32))

    # Increment LIDAR scan count for this timestamp
    systemstate.lidar_scan_index += 1
end

"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleGPS!(msg::sensor_msgs.msg.NavSatFix, fg::AbstractDFG, systemstate::SystemState)
    @info "handleGPS" maxlog=10
    if systemstate.cur_variable === nothing
        # Keyed by the radar, skip if we don't have a variable yet.
        return nothing
    end
    timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/10^9
    # Update the variable if needed
    # updateVariableIfNeeded(fg, systemstate, timestamp)
    @info "[$timestamp] GPS sample on $(systemstate.cur_variable.label)"
    
    if :GPS in listDataEntries(fg, systemstate.cur_variable.label)
        @warn "GPS sample on $(systemstate.cur_variable.label) already exist, dropping"
        return nothing 
    end

    ade,adb = addData!(fg, :gps_fix, systemstate.cur_variable.label, :GPS, Vector{UInt8}(JSON2.write(msg)),  mimeType="application/json")

end

function main()
    dfg_datafolder = "/tmp/rex"

    @info "Hit CTRL+C to exit and save the graph..."

    init_node("rex_feed")

    # Initialization
    fg = initfg()

    ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
    addBlobStore!(fg, ds)

    ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
    addBlobStore!(fg, ds)

    # add if you want lidar also 
    ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
    addBlobStore!(fg, ds)

    # System state
    systemstate = SystemState()


    # Enable and disable as needed.
    # Skipping LIDAR because those are huge...
    radar_sub = Subscriber{seagrant_msgs.msg.radar}("/radar_0", handleRadar!, (fg, systemstate), queue_size = 10)
    # radarpc_sub = Subscriber{sensor_msgs.msg.PointCloud2}("/radar_pointcloud0", handleRadarPointcloud!, (fg, systemstate), queue_size = 10)
    # lidar_sub = Subscriber{sensor_msgs.msg.PointCloud2}("/velodyne_points", handleLidar!, (fg,systemstate), queue_size = 10)
    gps_sub = Subscriber{sensor_msgs.msg.NavSatFix}("/gps/fix", handleGPS!, (fg, systemstate), queue_size = 10)

    @info "subscribers have been set up; entering main loop"
    loop_rate = Rate(20.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end

    @info "Exiting"
    # After the graph is built, for now we'll save it to drive to share.
    # Save the DFG graph with the following:
    @info "Saving DFG to $dfg_datafolder/dfg"
    saveDFG(fg, "$dfg_datafolder/dfg")

end

main()


## after the graph is saved it can be loaded and the datastores retrieved

dfg_datafolder = "/tmp/rex"

fg = loadDFG("$dfg_datafolder/dfg")

ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
addBlobStore!(fg, ds)

# add if you want lidar also 
ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
addBlobStore!(fg, ds)