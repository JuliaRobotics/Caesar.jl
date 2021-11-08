"""
    Proof of concept for Caesar-ROS integration
    (check Caesar Docs for details)
    https://juliarobotics.org/Caesar.jl/latest/examples/using_ros/

    Input:
    - Make sure the rosbag is in ~/data/Marine/philos_car_far.bag

    Output:
    - Generates output dfg tar and data folder at /tmp/caesar/philos 
        containing data from the bagfile, see below for details.

    Prerequisites:
    - source /opt/ros/noetic/setup.bash
    - cd ~/thecatkin_ws
        - source devel/setup.bash in all 3 terminals
    - run roscore in one terminal
    - Then run this Julia in another terminal/process.


    To do:
    - re-enable JSON replies
        s- periodic export of factor graph object
"""

## Prepare python version
using Distributed
# addprocs(4)

using Pkg
Distributed.@everywhere using Pkg

Distributed.@everywhere begin
  ENV["PYTHON"] = "/usr/bin/python3"
  Pkg.build("PyCall")
end

using PyCall
Distributed.@everywhere using PyCall

## INIT
using RobotOS

# Also rosnode info
# standard types
@rosimport sensor_msgs.msg: PointCloud2
@rosimport sensor_msgs.msg: NavSatFix
# @rosimport nmea_msgs.msg: Sentence
# seagrant type

# Application specific ROS message types from catkin workspace
# @rosimport seagrant_msgs.msg: radar

rostypegen()
# No using needed because we're specifying by full name.
# using .sensor_msgs.msg
# using .seagrant_msgs.msg

## Load Caesar with additional tools

using Colors
using Caesar

##

# using RoME
# using DistributedFactorGraphs

using DistributedFactorGraphs.DocStringExtensions
using Dates
using JSON2
using BSON
using Serialization
using FixedPointNumbers
using StaticArrays
using ImageMagick, FileIO

using ImageDraw

##

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
Base.@kwdef mutable struct SystemState
    curtimestamp::Float64 = -1000
    cur_variable::Union{Nothing, DFGVariable} = nothing
    var_index::Int = 0
    lidar_scan_index::Int = 0
    max_lidar::Int = 3
    radar_scan_queue::Channel{sensor_msgs.msg.PointCloud2} = Channel{sensor_msgs.msg.PointCloud2}(60)
    # SystemState() = new(-1000, nothing, 0, 0, 3)
end

##

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

function makeImage!(pc::Caesar._PCL.PointCloud,
                    x_domain::Tuple{<:Real,<:Real}=(-1000,1000),
                    y_domain::Tuple{<:Real,<:Real}=x_domain;
                    rows::Integer=1000, 
                    cols::Integer=rows,
                    img::AbstractMatrix{<:Colorant} = Gray.(zeros(UInt8,rows,cols)),
                    circle_size::Real=1,
                    drawkws... )
    #
    gridsize_x = (x_domain[2]-x_domain[1])/rows
    gridsize_y = (y_domain[2]-y_domain[1])/cols
    
    for pt in pc.points
        _x = trunc(Int,pt.x / gridsize_x + 500)
        _y = trunc(Int,pt.y / gridsize_y + 500)
        draw!( img, Ellipse(CirclePointRadius(_x,_y, circle_size)); drawkws... )  #; thickness = 1, fill = true)) )
    end
    
    return img
end


"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleRadarPointcloud!(msg::sensor_msgs.msg.PointCloud2, fg::AbstractDFG, systemstate::SystemState)
    @info "handleRadarPointcloud" maxlog=10

    # assume there is still space (previously cleared)
    # add new piece of radar point cloud to queue for later processing.
    put!(systemstate.radar_scan_queue, msg)

    # check if the queue still has space
    if length(systemstate.radar_scan_queue.data) < systemstate.radar_scan_queue.sz_max
        # nothing more to do
        return nothing
    end
    
    # Full sweep, lets empty the queue and add a variable
    # type instability
    queueScans = Vector{Any}(undef, systemstate.radar_scan_queue.sz_max)

    # get the first
    md = take!(systemstate.radar_scan_queue)
    pc2 = Caesar._PCL.PCLPointCloud2(md)
    pc_cat = Caesar._PCL.PointCloud(pc2)
    
    queueScans[1] = pc2

    for i in 1:length(systemstate.radar_scan_queue.data)
        # something minimal, will do util for transforming PointCloud2 next
        println(i)
        md = take!(systemstate.radar_scan_queue)
        # @info typeof(md) fieldnames(typeof(md))
        pc2 = Caesar._PCL.PCLPointCloud2(md)
        pc_ = Caesar._PCL.PointCloud(pc2)
        pc_cat = cat(pc_cat, pc_; reuse=true)

        queueScans[i] = (pc2) 
    end

    # add a new variable to the graph
    timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
    systemstate.curtimestamp = timestamp
    systemstate.cur_variable = addVariable!(fg, Symbol("x$(systemstate.var_index)"), Pose2, timestamp = unix2datetime(timestamp))
    systemstate.var_index += 1

    io = IOBuffer()
    serialize(io, queueScans)

    # @show datablob = pc # queueScans
    # and add a data blob of all the scans
    # Make a data entry in the graph
    addData!(   fg, :radar, systemstate.cur_variable.label, :RADAR_PC2s, 
                take!(io), # get base64 binary
                # Vector{UInt8}(JSON2.write(datablob)),  
                mimeType="application/octet-stream/julia.serialize",
                description="queueScans = Serialization.deserialize(PipeBuffer(readBytes))")
    #

    io = IOBuffer()
    serialize(io, pc_cat)

    addData!(   fg, :radar, systemstate.cur_variable.label, :RADAR_SWEEP,
                take!(io),
                mimeType="application/octet-stream/julia.serialize",
                description="queueScans = Serialization.deserialize(PipeBuffer(readBytes))" )
    #

    # also make and add an image of the radar sweep
    img = makeImage!(pc_cat)
    addData!(   fg, :radar, systemstate.cur_variable.label, :RADAR_IMG,
                Caesar.toFormat(format"PNG", img),
                mimeType="image/png",
                description="ImageMagick.readblob(imgBytes)" )
    #

    nothing
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

    io = IOBuffer()
    JSON2.write(io, msg)
    ade,adb = addData!(fg, :gps_fix, systemstate.cur_variable.label, :GPS, take!(io),  mimeType="application/json", description="JSON2.read(IOBuffer(datablob))")

end

## Own unpacking of ROS types from bagreader (not regular subscriber)


# TODO consolidated with RobotOS.jl pattern
_unpackROSMsgType(T::Type, msgdata) = convert(T, msgdata[2])

function _handleRadarPointcloud!(msgdata, args::Tuple)
    msgT = _unpackROSMsgType(sensor_msgs.msg.PointCloud2, msgdata)
    handleRadarPointcloud!(msgT, args...)
end

function _handleLidar!(msgdata, args::Tuple)
    msgT = _unpackROSMsgType(sensor_msgs.msg.PointCloud2, msgdata)
    handleLidar!(msgT, args...)
end

function _handleGPS!(msgdata, args)
    msgT = _unpackROSMsgType(sensor_msgs.msg.NavSatFix, msgdata)
    handleGPS!(msgT, args...)
end

##

function main(;iters::Integer=50)
    dfg_datafolder = "/tmp/caesar/philos"
    if isdir(dfg_datafolder)
        println("Deleting old contents at: ",dfg_datafolder)
        rm(dfg_datafolder; force=true, recursive=true)
    end
    mkdir(dfg_datafolder)

    @info "Hit CTRL+C to exit and save the graph..."

    init_node("asv_feed")
    # find the bagfile
    bagfile = joinpath(ENV["HOME"],"data","Marine","philos_car_far.bag")
    bagSubscriber = RosbagSubscriber(bagfile)

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
    # radar_sub = Subscriber{seagrant_msgs.msg.radar}("/radar_0", handleRadar!, (fg, systemstate), queue_size = 10)
    # gps_sub = Subscriber{sensor_msgs.msg.NavSatFix}("/gps/fix", handleGPS!, (fg, systemstate), queue_size = 10)
    
    # radar_sub = bagSubscriber(, _handleRadar!, (fg, systemstate))

    radarpc_sub = bagSubscriber("/broadband_radar/channel_0/pointcloud", _handleRadarPointcloud!, (fg, systemstate) )
    # lidar_sub = Subscriber{sensor_msgs.msg.PointCloud2}("/velodyne_points", handleLidar!, (fg,systemstate), queue_size = 10)
    gps_sub = bagSubscriber("/gnss", _handleGPS!, (fg, systemstate))


    @info "subscribers have been set up; entering main loop"
    # loop_rate = Rate(20.0)
    while loop!(bagSubscriber)
        iters -= 1
        iters < 0 ? break : nothing
    end

    @info "Exiting"
    # After the graph is built, for now we'll save it to drive to share.
    # Save the DFG graph with the following:
    @info "Saving DFG to $dfg_datafolder/dfg"
    saveDFG(fg, "$dfg_datafolder/dfg")

end

##


main(iters=1500)


## after the graph is saved it can be loaded and the datastores retrieved

dfg_datafolder = "/tmp/caesar/philos"

fg = loadDFG("$dfg_datafolder/dfg")

ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
addBlobStore!(fg, ds)

# add if you want lidar also 
ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
addBlobStore!(fg, ds)


## load one of the PointCloud sets

de,db = getData(fg, :x0, :RADAR_SWEEP)

sweep = deserialize(PipeBuffer(db)) # BSON.@load


## 

using Gadfly
Gadfly.set_default_plot_size(40cm,20cm)


##

lb = :x0
de,db = getData(fg, lb, :RADAR_SWEEP)
pointcloud = deserialize(PipeBuffer(db)) 

X = (c->c.x).(pointcloud.points)
Y = (c->c.y).(pointcloud.points)

##

PL = []
push!(PL, Gadfly.layer(x=X, y=Y, Geom.point))

Gadfly.plot(PL...)

## Converting PCLPointCloud2 again


PL = []

for lb in [:x0;] #:x1;:x2;:x3;:x4;:x5]
    de,db = getData(fg, lb, :RADAR_PC2s)
    queueScans = deserialize(PipeBuffer(db)) 
    for pc2 in queueScans[1:end]
        pc_ = Caesar._PCL.PointCloud(pc2)
        X = (c->c.x).(pc_.points)
        Y = (c->c.y).(pc_.points)
        push!(PL, Gadfly.layer(x=X, y=Y, Geom.point))
    end
end

Gadfly.plot(PL...)

##








##