# Victoria Park

using HDF5, JLD, Gadfly, Colors, Cairo
using KernelDensityEstimate, Distributions
using Caesar, IncrementalInference, RoME

# load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder
include(joinpath(dirname(@__FILE__),"loadVicPrkData.jl"))



T=30 # 1400
fg = Caesar.initfg();
idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:unimodal, MM=MMr);


# Start with a fresh factor graph
fg = Caesar.initfg();

# insert the first pose node in two dimensions, that is [x, y, theta]

# add prior information as a factor to X1 is found in d[1]...
d[1]

# Go look at what this function,
#  function appendFactorGraph!(fg::FactorGraph,
# is doing and see if you can build the new pose functions line by line, similar to
# what you had done in the first ROV example.


# you should have one node X1 with a solid dot prior factor attached
Graphs.plot(fg.g)

# you can also draw the result with plotting using
drawPoses(fg)
# # alternative
# vc = startdefaultvisualization()
# visualizeallposes!(vc, fg, drawlandms=false)


# now add odometry, using information from d[2] for relative constaint to X2
# ... add node here
# delta x,, delta y, delta yaw, *, *
d[2]


# ...
d[3]



# coninue adding poses up to pose X10
