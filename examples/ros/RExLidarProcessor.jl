using DistributedFactorGraphs
using IncrementalInference, RoME

# Example of a quick processor.

# Set this
dfgDataFolder = "/tmp/rex"

# Load the graph
fg = initfg()
loadDFG("$dfgDataFolder/dfg", IncrementalInference, fg)

# Check what's in it
ls(fg)
lsf(fg)
# How about bigdata entries?
count(v -> :RADAR in getBigDataKeys(v), getVariables(fg))
count(v -> :LIDAR in getBigDataKeys(v), getVariables(fg))
allLidarVariables = filter(v -> :LIDAR in getBigDataKeys(v), getVariables(fg))

# Reopen the file data store
datastore = FileDataStore("$dfgDataFolder/bigdata")
# Let's just use the first key for testing
var = allLidarVariables[1]
entry = getBigDataEntry(var, :LIDAR)
rawData = getBigData(datastore, entry)

# Okay, all loaded.

### Quick example of working with the data...

"""
Quick function to extract a frame of Float32's from raw data.
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
#radarData = extractDataFrame(rawData, 3, [1,2,3])

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
