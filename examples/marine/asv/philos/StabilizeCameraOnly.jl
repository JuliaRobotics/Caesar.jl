
#=

Draw radar map in resulting world frame (after solve)


=#

##


using Distributed
# addprocs(10)

using Colors
using Caesar
@everywhere using Caesar

using DistributedFactorGraphs: listDataEntrySequence

import Rotations as _Rot
using CoordinateTransformations

# using DistributedFactorGraphs.DocStringExtensions
using Dates
using JSON2
# using BSON
using Serialization
using FixedPointNumbers
using Manifolds
using StaticArrays
using TensorCast
using DataStructures

using ImageMagick, FileIO
using Images
using ImageDraw
using ImageView
using ImageTransformations
using ImageFeatures

using VideoIO

using RoMEPlotting
Gadfly.set_default_plot_size(35cm,20cm)

using ProgressMeter

using Optim

## load the solved graph and reattach the datastores

dfg_datafolder = "/tmp/caesar/philos"

fg = loadDFG("$dfg_datafolder/results_5/x0_5_x280")

##

ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:camera, "$dfg_datafolder/data/camera")
addBlobStore!(fg, ds)


##

M = SpecialEuclidean(2)
e0 = identity_element(M)

##


##======================================================================================

include(joinpath(@__DIR__,"ImageOnlyStabilization.jl"))



##

lbls = sortDFG(ls(fg, tags=[:POSE;])) #[1:100]

_stabImageSeq(fg, lbls; filecount=20, threshold=0.2, kparg1=12, size=512, window=20, logentry=true, N=15, scale_r=0.8)
# _stabImageSeq(fg; filecount=11, threshold=0.2, kparg1=10, logentry=true)

##

saveDFG("$dfg_datafolder/results_5/x0_5_x280_stab", fg)


##


# count = 0
# let count = count
#   @showprogress 1 "sweep" for kp1=[12;], thr=[0.2;0.5;0.8;1.1;]
#     count += 1
#     _stabImageSeq(fg, lbls; filecount=count, threshold=thr, kparg1=kp1, logentry=true)
#   end
# end

##

# T = typeof(IMGS[1])
# IMGS_ = Vector{T}(undef, length(IMGS))
# IMGS_ .= IMGS
# writevideo("/tmp/caesar/philos/quickstab.mp4", IMGS_; fps=90)

##

# mosaicMatches(imgG[[1;3]]..., matches) |> imshow
# trans, matches = calcMatchMinimize(imgG[[1;2]]...); trans

##

# img1 = Gray.(imgs[1])
# img2 = Gray.(imgs[3])
# tr, matches = calcMatchMinimize(img1,img2; threshold=0.4, size=1024, window=50)
# mosaicMatches(img1, img2, matches) |> imshow



##


#