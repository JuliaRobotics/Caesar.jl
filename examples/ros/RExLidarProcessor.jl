









# let's try to fetch variables with fullsweeps in them
allSweepVariables = filter(v -> :RADARSWEEP in getBigDataKeys(v), getVariables(fg));

saveDFG(fg, "$dfgDataFolder/fullsweep.tar.gz")

# Okay... see you lidar!

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
