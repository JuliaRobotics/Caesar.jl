
#=

Draw radar map in resulting world frame (after solve)


=#

##


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

fg = loadDFG("$dfg_datafolder/results_4/x0_5_x280")

##

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
img0 = makeImage!(wPC0, (-1500,1500); color=Gray{N0f8}(0.01)) 
imshow(img0)

wPC50 = _fetchDataPointCloudWorld(fg, :x50)
# makeImage!(wPC50, (-1500,1500)) |> imshow

wPC100 = _fetchDataPointCloudWorld(fg, :x100)
# makeImage!(wPC100, (-2000,2000)) |> imshow


##

_X_ = 3000
_Y_ = 1250

wPC__ = _fetchDataPointCloudWorld.(fg, [Symbol(:x,i) for i in 0:5:280]);
img_wPC_ = map(pc->makeImage!(pc, (-1250,_X_),(-_Y_,_Y_); color=Gray{Float64}(0.05)), wPC__);
img_wPC = +(img_wPC_...);

imshow(img_wPC)

## movie sequence in world frame

IMGS = Vector{Matrix{RGB{Float64}}}()

img_wPC = RGB.(img_wPC_[1])
for gim in img_wPC_[2:end]
  # green overlay of latest
  imgG = (px->RGB(0,10*px,0)).(gim);
  imgL = img_wPC + imgG
  push!(IMGS, imgL)

  # grow full world map
  img_wPC += RGB.(gim)
end

writevideo("/tmp/caesar/philos/results_4/radarmap.mp4", IMGS; fps=3, player="totem")




## ================================================================================
## Keeping a few code fragments below
## ================================================================================


# for l in [Symbol(:x,i) for i in 10:10:280]
#   wPC = cat(wPC, _fetchDataPointCloudWorld(fg, l); reuse=true )
# end
# img_wPC = makeImage!(wPC, (-1250,_X_),(-_Y_,_Y_)) 
# imshow(img_wPC)

## manually north align

# nPC = apply(M, ProductRepr([0.0;0.0], _Rot.RotMatrix(135*pi/180.0)), wPC)

# img_nPC = makeImage!(nPC, (-2500,1000),(-2000,1500), color=Gray{N0f8}(0.1)) 
# img_nPC[1,1] = Gray(1)
# imshow(img_nPC)

# ##

# using ImageMagick, FileIO
# pngimg = toFormat(format"PNG", img_wPC)
# save(dfg_datafolder*"/results_3/wPC.png", img_wPC)

# ##


#