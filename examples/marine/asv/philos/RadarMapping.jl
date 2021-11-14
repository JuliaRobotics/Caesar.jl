
#=

Draw radar map in resulting world frame (after solve)


=#



using Distributed
# addprocs(10)

using Colors
using Caesar
@everywhere using Caesar

import Rotations as _Rot

# using DistributedFactorGraphs.DocStringExtensions
using Dates
using JSON2
# using BSON
using Serialization
using FixedPointNumbers
using Manifolds
using StaticArrays
using ImageMagick, FileIO
using Images

using ImageDraw
using ImageView

using RoMEPlotting
Gadfly.set_default_plot_size(35cm,20cm)


## load the solved graph and reattach the datastores

dfg_datafolder = "/tmp/caesar/philos"

fg = loadDFG("$dfg_datafolder/results_3/x0_5_x280")

ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
addBlobStore!(fg, ds)

# add if you want lidar also 
ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
addBlobStore!(fg, ds)


## visualize the radar data

if false
  imgs = map(x->Gray{N0f8}.(x), fetchDataImage.(fg, sortDFG(ls(fg)), :RADAR_IMG));
  writevideo("/tmp/caesar/philos/radar280.mp4", imgs; fps=5, player="totem")
end

##

M = SpecialEuclidean(2)
e0 = identity_element(M)

##


# get two radar sweep point clouds and rotate them to the world frame
function _fetchDataPointCloudWorld(fg::AbstractDFG, lb::Symbol)
  de,db = getData(fg, lb, :RADAR_SWEEP)
  # deserialize is not legacy safe, rather use BSON.@save(io, Caesar._PCL.PCLPointCloud2(pc))
  bPC = deserialize(PipeBuffer(db))
  wPb = exp(M, e0, hat(M, e0, getPPE(fg, lb).suggested))
  apply(M, wPb, bPC)
end


##

wPC0 = _fetchDataPointCloudWorld(fg, :x0)
# makeImage!(wPC0, (-1500,1500)) |> imshow

wPC50 = _fetchDataPointCloudWorld(fg, :x50)
# makeImage!(wPC50, (-1500,1500)) |> imshow

wPC100 = _fetchDataPointCloudWorld(fg, :x100)
# makeImage!(wPC100, (-2000,2000)) |> imshow


##

wPC = _fetchDataPointCloudWorld(fg, :x0)
for l in [Symbol(:x,i) for i in 10:10:280]
  wPC = cat(wPC, _fetchDataPointCloudWorld(fg, l); reuse=true )
end
# wPC = cat(wPC0, wPC50, wPC100)

_X_ = 3000
_Y_ = 1250
img_wPC = makeImage!(wPC, (-1250,_X_),(-_Y_,_Y_)) 
imshow(img_wPC)

## manually north align


nPC = apply(M, ProductRepr([0.0;0.0], _Rot.RotMatrix(135*pi/180.0)), wPC)

img_nPC = makeImage!(nPC, (-2500,1000),(-2000,1500)) 
imshow(img_nPC)

##

using ImageMagick, FileIO

pngimg = toFormat(format"PNG", img_wPC)

save(dfg_datafolder*"/results_3/wPC.png", img_wPC)

##


#