"""
This script shows how to assemble full 360 degree radar sweeps from the DFG object
built with RExFeed.jl
"""
    
using DistributedFactorGraphs
using IncrementalInference, RoME
using JSON2
using LinearAlgebra

##

# Where to fetch data
# dfgDataFolder = ENV["HOME"]*"/data/rex";
dfgDataFolder = "/tmp/caesar/rex"

# Load the graph
fg = loadDFG("$dfgDataFolder/dfg")

# add the datastore locations
ds = FolderStore{Vector{UInt8}}(:radar, "$dfgDataFolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfgDataFolder/data/gps")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:lidar, "$dfgDataFolder/data/lidar")
addBlobStore!(fg, ds)

##

# Check what's in it
ls(fg)
lsf(fg)  # there should be no factors the first time a session is loaded

# How about radar data entries?
count(v -> :RADAR in listDataEntries(v), getVariables(fg))

# may be too intense as it is fetching ALL data too?
allRadarVariables = filter(v -> :RADAR in listDataEntries(v), getVariables(fg));

allRadarVariables = sortDFG(allRadarVariables);


function fetchRadarMsg(var::DFGVariable)
    entry,rawData = getData(fg, var.label, :RADAR)
    # raw data is json-encoded; this decoding should happen inside getBigData?
    return JSON2.read(IOBuffer(rawData))
end

msg = fetchRadarMsg(allRadarVariables[1])

##

# function azimuth(msg::NamedTuple)
function azimuth(msg)
    """
    Assemble azimuth vector

    This function assembles a vector of azimuth angles for the given ping. It
    handles both the conversion from [0,360) to [-pi,pi), and the wrap around
    scenario when msg.angle_start > msg.angle_end (or, equivalently,
    msg.angle_increment != 0.17578125).

    """
    if msg.angle_start > msg.angle_end
        # wrap around; increments become messy
        delta = (360 - msg.angle_start) + (msg.angle_end)
        angle_step = delta/msg.n
        a = Vector(range(msg.angle_start, msg.angle_start+delta,length = msg.n))
        a[a.>360].-=360.
        #   sa = [Vector(range(msg.angle_start, 360.0, step = 0.17578125)); Vector(reverse(range(msg.angle_end, 0.,step=-0.17578125)))]
        # use reverse to ensure we stop at the correct angle
    else
        a = range(msg.angle_start, msg.angle_end, length=msg.n)
    end

    # @show a
    a = deg2rad.(Vector(a))   # to radians
    a[a.>pi]=a[a.>pi].-(2*pi) # normalize to [-pi,pi)

    return (a)
end

function nearest(v::Vector{<:Real}, q::Real)::Int64
    return findmin(abs.(v.-q))[2]
end

function tocartsector(ping_polar::Array{Any,2},r::Vector{Float64}, a::Vector{Float64}, res::Float64 )
    """
    res - resolution, in m/px
    """

    vx = [r[1]*cos(a[1]), r[1]*cos(a[end]), r[end]*cos(a[1]), r[end]*cos(a[end])];
    vy = [r[1]*sin(a[1]), r[1]*sin(a[end]), r[end]*sin(a[1]), r[end]*sin(a[end])];
    xmin =  minimum(vx)
    xmax = maximum(vx)
    nx = Int64(round((xmax-xmin)/res))
    x = Vector(range(xmin, xmax, length=nx))

    ymin = minimum(vy)
    ymax = maximum(vy)
    ny = Int64(round((ymax-ymin)/res))
    y = Vector(range(ymin, ymax, length=ny))

    ping_cart = zeros(nx, ny)

    for i = 1:nx
        for j=1:ny
            rq = norm([x[i],y[j]])
            if rq > r[end] || rq < r[1]
                continue
            end

            aq = atan(y[j],x[i])
            if aq > a[end] || aq < a[1]
                continue
            end

            # idx_r = Int(ceil((rq-r[1])/(r[end]-r[1])))
            # idx_a = Int(ceil((aq-a[1])/(a[end]-a[1]))) # dangerous!
            # pvt: turns out that using nearest, while arguably more readable, results in a significant performance hit!
            #############
            # w/ nearest:
            # memory estimate:  1.28 GiB
            # allocs estimate:  1386472
            # --------------
            # minimum time:     797.162 ms (22.86% GC)
            # median time:      960.964 ms (22.70% GC)
            # mean time:        962.183 ms (22.91% GC)
            # maximum time:     1.208 s (22.71% GC)
            # --------------
            # samples:          6
            # evals/sample:     1
            ##############
            # w/o nearest:
            # memory estimate:  72.06 MiB
            # allocs estimate:  922543
            # --------------
            # minimum time:     76.095 ms (19.39% GC)
            # median time:      78.501 ms (20.36% GC)
            # mean time:        94.679 ms (33.84% GC)
            # maximum time:     227.771 ms (70.20% GC)
            # --------------
            # samples:          53
            # evals/sample:     1
            idx_r =  nearest(r,  rq)
            idx_a = nearest(a, aq)
            ping_cart[i,j] = ping_polar[idx_r, idx_a]
        end
    end
    return ping_cart
end

function tocart(ping_polar::Array{Any,2},r::Vector{<:Real}, a::Vector{<:Real}, res::Real )
    """
    res - resolution, in m/px
    """

    n = Int64(round((2*r[end]/res)))
    x = Vector(range(-r[end], r[end], length=n))
    y = Vector(range(-r[end], r[end], length=n))
    ping_cart = zeros(n, n)

    for i = 1:n
        for j=1:n
            rq = norm([x[i],y[j]])
            if rq > r[end] || rq < r[1]
                continue
            end

            aq = atan(y[j],x[i])
            if aq > a[end] || aq < a[1]
                continue
            end

            idx_r =  nearest(r,  rq)
            idx_a = nearest(a, aq)
            ping_cart[i,j] = ping_polar[idx_r, idx_a]
        end
    end
    return ping_cart
end

# function assembleping(msg::NamedTuple, resolution::Float64)
function assembleping(msg, resolution::Float64)
    """
    Assemble polar and cartesian pings from radar message.
    """
    ping_polar= reshape(msg.radar_data, (msg.m, msg.n))

    # assemble azimuth and range vectors
    a = azimuth(msg)
    r = Vector(range(msg.range_min, msg.range_max, length=msg.m ))

    # option a: tight cartesian around the sector
    # ping_cart = tocartsector(ping_polar, r, a, resolution)

    # option b: full 360
    ping_cart = tocart(ping_polar, r, a, resolution)

    return (ping_polar, ping_cart)
end

# msg = alldata[1]
# (polar, cart) = assembleping(msg, 2.0);
# fullsweep = round.(UInt8, cart./255)
# imi = imshow(fullsweep)

# for i = 1:20
#     @sync begin
#         @async begin
#             println(i)
#             if mod(i, 10) == 0
#                 fullsweep .&= 0x00
#             end
#             # sleep(0.1)
#         end
#         @async begin
#             msg = alldata[i]
#             # set resolution to 1m/px
#             (polar, cart) = assembleping(msg, 2.0);
#             fullsweep .|= round.(UInt8, cart./255)
#             imshow!(imi["gui"]["canvas"], fullsweep) 
#         end
#     end
# end

##

# set resolution to 1m/px
(polar, cart) = assembleping(msg, 1.0);


using ImageMagick
using Images, ImageView
using ImageFiltering
# imshow(polar)
# imshow(cart)

##

output_dir = joinpath(dfgDataFolder,"pings")
if (!isdir(output_dir))
    mkdir(output_dir)
end
save(joinpath(output_dir,"ping.png"),UInt8.(cart))

##
# now let's process all radar variables in here

fullsweep = zeros(size(cart))
last_angle = 0.
sweep = 0;

for  i in 1:length(allRadarVariables)
    msg = fetchRadarMsg(allRadarVariables[i])

    delta_angle = msg.angle_increment
    # TODO: check upper bound; may have valid but coarse sectors
    if (msg.angle_increment<0 && msg.angle_increment > 0.2)
        # wrap around issues caused by driver: angle_increment
        println("Angle increment exceeds bounds (",msg.angle_increment,"); using default value")
        delta_angle = 0.17578125;
    end

    println(String(allRadarVariables[i].label)," - start:",msg.angle_start,", stop:",msg.angle_end, " step:",msg.angle_increment, " steps:",msg.n)
    (polar, cart) = assembleping(msg, 1.0)
    global fullsweep # need global due to julia's scope handling
    global last_angle
    global sweep
    fullsweep = fullsweep + cart

    # TODO: de-rotate angles by vehicle heading, so that test checks for full sweep in world frame
    if (msg.angle_end < msg.angle_start)# || msg.angle_start<last_angle)
        println("!")
        # now that we have a full sweep, we can try to match with the previous one!
        # TODO: register w/  previous sweep
        # save to disk
        fname = join(["full_",string(sweep),".png"])
        save(joinpath(output_dir,fname),UInt8.(clamp.(fullsweep,0,255)))

        # add full sweep to last variable
        ade,adb = addData!(fg, :radar, allRadarVariables[i].label, :RADARSWEEP, Vector{UInt8}(JSON2.write(fullsweep)), mimeType="application/json")

        # reset sweep
        fullsweep = zeros(size(cart))
        sweep = sweep+1
    end
    # fname = join(["polar_",String(allRadarVariables[i].label),".png"])
    # save(joinpath(output_dir,fname),UInt8.(polar))
    fname = join(["cart_",String(allRadarVariables[i].label),".png"])
    save(joinpath(output_dir,fname),UInt8.(cart))
    last_angle = msg.angle_end
end
    


##


# count(v -> :RADARSWEEP in listDataEntries(v), getVariables(fg))

# # may be too intense as it is fetching ALL data too?
# allSweepVariables = filter(v -> :RADARSWEEP in listDataEntries(v), getVariables(fg));

# allSweepVariables = sortDFG(allSweepVariables)


# getLabel.(allSweepVariables)


# de, db = getData(fg, allSweepVariables[1].label, :RADARSWEEP)

# JSON2.read(IOBuffer(db))


#