"""
this script fetches sequential pairs of poses, fetches the big data (radar pings) tied to those poses, and then determines the pairwise factors that should be added between these sequential pairs
"""

using Images
using Caesar
using JSON2
using Manifolds

using ImageView

# import Rotations as _Rot

# using Caesar: ScatterAlignPose2


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

# fetch variables containing a full sweep
allSweepVariables = filter(v -> :RADARSWEEP in listDataEntries(v), getVariables(fg)) |> sortDFG

fsvars = allSweepVariables .|> getLabel

# helper function to retrieve the radar sweep for a given variable
function fetchSweep(dfg::AbstractDFG, varlabel::Symbol)
  #
  entry,rawData = getData(dfg, varlabel, :RADARSWEEP)
  rawdata = Vector{Float64}(JSON2.read(IOBuffer(rawData)))
  n = Int(sqrt(length(rawdata)))
  sweep = reshape(rawdata,(n,n))
  return sweep # That's pretty sweep if i say so myself...
end

##

# fetch all radar pings
sweeps = fetchSweep.(fg, fsvars);

# Filter the images
kg = Kernel.gaussian(7)
sweeps = map(s -> imfilter(s, kg), sweeps)
# Normalize
sweeps = map(s -> s/maximum(s), sweeps);

# At this point we can load the sweeps; let's work on registration
# First step is to have a function that evaluates the cost of a given transform
# between two subsequent images.


## Building the graph

# maybe one exists, or build a new one below
# newfg = loadDFG("$dfgDataFolder/newfg")

startsweep = 5
endsweep = 26
graphinit = true

len = size(sweeps[5],1)
domain = (range(-100,100,length=976),range(-100,100,length=976))

## use a standard builder

STEP = 20
sweeps_ = sweeps[startsweep:STEP:endsweep]

# build the graph
newfg = buildGraphChain!( sweeps_,
                          ScatterAlignPose2,
                          (_, data) -> (data.currData,data.nextData,domain),
                          stopAfter=1,
                          doRef = false,
                          inflation_fct=0.0,
                          solverParams=SolverParams(graphinit=true, inflateCycles=1) )
#



##


# Save the graph
saveDFG("$dfgDataFolder/segment_test.tar.gz", newfg);

##

lsf(newfg)
# this should run the radar alignment
X1_ = approxConvBelief(newfg, :x0x1f1, :x1)
pts = getPoints(X1_)

# solving will internally call initAll!(newfg)
tree = solveTree!(newfg)


##

using Cairo, Gadfly
Gadfly.set_default_plot_size(40cm,20cm)


#