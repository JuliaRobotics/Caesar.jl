using DistributedFactorGraphs
using IncrementalInference, RoME;
using JSON2;

# Where to fetch data
dfgDataFolder = "/tmp/rex";

# Load the graph
fg = initfg()
loadDFG("$dfgDataFolder/dfg", IncrementalInference, fg);

# Check what's in it
ls(fg)
lsf(fg)  # there should be no factors the first time a session is loaded

# How about bigdata entries?
count(v -> :RADAR in getBigDataKeys(v), getVariables(fg));
count(v -> :LIDAR in getBigDataKeys(v), getVariables(fg));

# may be too intense as it is fetching ALL data too?
allRadarVariables = filter(v -> :RADAR in getBigDataKeys(v), getVariables(fg));

# sort variables by timestamp (deprecated in DFG 0.6.0)
import Base.isless

function isless(a::DFGVariable, b::DFGVariable)
    return isless(a.timestamp,b.timestamp)
end
allRadarVariables = sort(allRadarVariables)

function fetchRadarMsg(var::DFGVariable, store::FileDataStore)
    entry = getBigDataEntry(var, :RADAR)
    rawData = getBigData(datastore, entry)
    # raw data is json-encoded; this decoding should happen inside getBigData?
    return JSON2.read(IOBuffer(rawData))
end

datastore = FileDataStore("$dfgDataFolder/bigdata")
msg = fetchRadarMsg(allRadarVariables[1], datastore)

function azimuth(msg::NamedTuple)
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

    @show a
    a = deg2rad.(Vector(a))   # to radians
    a[a.>pi]=a[a.>pi].-(2*pi) # normalize to [-pi,pi)

    return (a)
end

function nearest(v::Vector{Float64}, q::Float64)::Int64
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

function tocart(ping_polar::Array{Any,2},r::Vector{Float64}, a::Vector{Float64}, res::Float64 )
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

function assembleping(msg::NamedTuple, resolution::Float64)
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

# Now let's extract the radar sector measurement


# set resolution to 1m/px
(polar, cart) = assembleping(msg, 1.0);

using Images, ImageView
imshow(polar)
imshow(cart)

output_dir = joinpath(dfgDataFolder,"pings")
if (!isdir(output_dir))
    mkdir(output_dir)
end
save(joinpath(output_dir,"ping.jpg"),UInt8.(cart))


# now let's process all radar variables in here

fullsweep = zeros(size(cart))
last_angle = 0.
sweep = 0;

for  i in 1:length(allRadarVariables)
    msg = fetchRadarMsg(allRadarVariables[i], datastore)

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
        fname = join(["full_",string(sweep),".jpg"])
        save(joinpath(output_dir,fname),UInt8.(clamp.(fullsweep,0,255)))

        # add full sweep to last variable
        # Make a big data entry in the graph
        element = GeneralBigDataEntry(fg, allRadarVariables[i], :RADARSWEEP, mimeType="application/json")
        # Set it in the store
        addBigData!(datastore, element, Vector{UInt8}(JSON2.write(fullsweep)))
        # Add the entry to the graph
        addBigDataEntry!(allRadarVariables[i], element)

        # reset sweep
        fullsweep = zeros(size(cart))
        sweep = sweep+1
    end
    # fname = join(["polar_",String(allRadarVariables[i].label),".jpg"])
    # save(joinpath(output_dir,fname),UInt8.(polar))
    fname = join(["cart_",String(allRadarVariables[i].label),".jpg"])
    save(joinpath(output_dir,fname),UInt8.(cart))
    last_angle = msg.angle_end
end



# let's try to fetch variables with fullsweeps in them
allSweepVariables = filter(v -> :RADARSWEEP in getBigDataKeys(v), getVariables(fg));

# try and display one of them
function fetchSweep(var::DFGVariable, store::FileDataStore)
    entry = getBigDataEntry(var, :RADARSWEEP)
    rawData = getBigData(datastore, entry)
    # raw data is json-encoded; this decoding should happen inside getBigData?
    return Vector{Float64}(JSON2.read(IOBuffer(rawData)))
end

sweep = fetchSweep(allSweepVariables[10], datastore)
n = Int(sqrt(length(s)))
sweep = reshape(sweep,(n,n))
imshow(sweep)

##############################################################################
########################### LASERS AND STUFF BELOW ###########################
##############################################################################

# Quick function to find the closest point as trivial example
# We're going to save this back into the data as 'processed data'
struct ClosestPoint
    cp::Vector{Float32}
    dist::Float32
end
function findClosestPoint(xyzData::Vector{Vector{Float32}})::ClosestPoint
    # Judgement free zone please :) just a hacky example
    dist = 1.0e6
    cp = [0,0,0]
    for p in xyzData
        d = sqrt(p[1]^2 + p[2]^2 + p[3]^2)
        if d < dist
            dist = d
            cp = p
        end
    end
    return ClosestPoint(cp, dist)
end

cp = findClosestPoint(lidarData)

### Quick version of writing back - write this back to our data.
# Serialize it using JSON2...
using JSON2
newData = Vector{UInt8}(JSON2.write(cp))

# Make an element and entry.
element = GeneralBigDataEntry(fg, var, :LIDARCP, mimeType="application/json")
# Set it in the store
addBigData!(datastore, element, newData)
# Make it in the graph
addBigDataEntry!(var, element)

# Save our graph with the new entries.
saveDFG(fg, "$dfgDataFolder/dfg")

# You can now load this later and retrieve that data.




### Quick example of working with the data...

"""
Quick function to extract a frame of Float32's from raw data.
(this is used for pointcloud data)
"""
function extractDataFrame(rawData::Vector{UInt8}, floatStepSize::Int, extractIndexes::Vector{Int})
    reformat = reinterpret(Float32, rawData)
    #Set up a map to get the indexes out
    dataset = map(i -> reformat[extractIndexes.+i], collect(0:(Int(length(reformat)/floatStepSize)-1))*floatStepSize)
    return dataset
end


# Now extract and format to our dataframe
lidarData = extractDataFrame(rawData, 8, [1,2,3])
# Radar would be similar (need to confirm that function works though):
# radarData = extractDataFrame(msg.radar_data, 3, [1,2,3])
