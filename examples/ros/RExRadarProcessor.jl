using DistributedFactorGraphs
using IncrementalInference, RoME

# Example of a quick processor.

# Set this
# dfgDataFolder = "/tmp/rex"
dfgDataFolder = joinpath(ENV["HOME"],"data","SeaGrant","fgfiles","02")

# Load the graph
fg = initfg()
loadDFG("$dfgDataFolder/dfg", Main, fg)

# Check what's in it
ls(fg)
lsf(fg)
# How about bigdata entries?
count(v -> :RADAR in getBigDataKeys(v), getVariables(fg))
count(v -> :LIDAR in getBigDataKeys(v), getVariables(fg))
# allLidarVariables = filter(v -> :LIDAR in getBigDataKeys(v), getVariables(fg))
allRadarVariables = filter(v -> :RADAR in getBigDataKeys(v), getVariables(fg))



# Reopen the file data store
datastore = FileDataStore("$dfgDataFolder/bigdata")
# Let's just use the first key for testing
# var = allLidarVariables[1]
# entry = getBigDataEntry(var, :LIDAR)
# rawData = getBigData(datastore, entry)
rvar = allRadarVariables[1]
entry = getBigDataEntry(rvar, :RADAR)
rawData = getBigData(datastore, entry)

#

## how to deserialize RADAR isMarginalized

string(rawData)


using JSON2

strData = String(take!(IOBuffer(rawData)))
dsr = JSON2.read(strData)

Vector{UInt8}(JSON2.write(msg))

dsr[:header]






#
