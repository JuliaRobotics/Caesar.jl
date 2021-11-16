
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
using ImageTransformations

using VideoIO

using RoMEPlotting
Gadfly.set_default_plot_size(35cm,20cm)


## load the solved graph and reattach the datastores

dfg_datafolder = "/tmp/caesar/philos"

fg = loadDFG("$dfg_datafolder/results_5/x0_5_x260")

##

ds = FolderStore{Vector{UInt8}}(:radar, "$dfg_datafolder/data/radar")
addBlobStore!(fg, ds)

ds = FolderStore{Vector{UInt8}}(:gps_fix, "$dfg_datafolder/data/gps")
addBlobStore!(fg, ds)

# add if you want lidar also 
ds = FolderStore{Vector{UInt8}}(:lidar, "$dfg_datafolder/data/lidar")
addBlobStore!(fg, ds)

# add if you want lidar also 
ds = FolderStore{Vector{UInt8}}(:camera, "$dfg_datafolder/data/camera")
addBlobStore!(fg, ds)


## structure

# walk through all the poses in fg
#  pull out all world frame radar images
#  pull out all camera images at each (not all poses have world location solution)
#  draw side by side radar on left and camera on right (more camera than radar images)
#  


## visualize the radar data

# if false
#   imgs = map(x->Gray{N0f8}.(x), fetchDataImage.(fg, sortDFG(ls(fg)), :RADAR_IMG));
#   writevideo("/tmp/caesar/philos/radar280.mp4", imgs; fps=5, player="totem")
# end

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

function listDataEntrySequence( fg::AbstractDFG,
                                lb::Symbol,
                                pattern::Regex,
                                _sort=(x)->x)
  #
  ents_ = listDataEntries(fg, lb)
  entReg = map(l->match(pattern, string(l)), ents_)
  entMsk = entReg .!== nothing
  ents_[findall(entMsk)] |> _sort
end

Base.clamp(img::AbstractMatrix{<:RGB},b::Real,t::Real) = map(px->RGB{N0f8}(clamp(px.r,b,t),clamp(px.g,b,t),clamp(px.b,b,t)), img)


function buildPoseImages( fg::AbstractDFG, 
                          lb::Symbol;
                          # _X_ = 3000,
                          # _Y_ = 1200,
                          T = RGB{N0f8},
                          color=Gray{Float64}(0.05),
                          scale_map=1.2 )
  #
  global img_wPC, imgL

  R_ = _Rot.RotMatrix(0.017*pi)

  framestack = []

  # fetch the camera images
  ents = listDataEntrySequence(fg, lb, r"IMG_CENTER_", sortDFG)
  imgs_lb = fetchDataImage.(fg, lb, ents)
  rows,cols = size(imgs_lb[1])
  
  # expand the radar map
  if isSolved(fg, lb)
    wPC__ = _fetchDataPointCloudWorld(fg, lb);
    nPC = apply(M, ProductRepr([0.0;0.0], _Rot.RotMatrix(145*pi/180.0)), wPC__)
    img_wPC_ = makeImage!(nPC, (-2000,1000),(-1500,1000); rows=trunc(Int, scale_map*rows), color);
    if lb == :x0
      img_wPC = deepcopy(img_wPC_)
    else
      img_wPC .+= img_wPC_
    end
    imgG = (px->RGB(0,10*px,0)).(img_wPC_);

    imgL = RGB.(img_wPC) + imgG
  end

  imgs_R = map(img->warp(img, ImageTransformations.recenter(R_, center(img))), imgs_lb)

  frame_ = Matrix{T}(undef, size(imgs_R[1])...)

  # write mosaic frames to the video
  for frame in imgs_R
    ax, ay = axes(frame)
    I = ax.parent .+ ax.offset
    J = ay.parent .+ ay.offset
    for (i,i_) in enumerate(I), (j,j_) in enumerate(J)
      frame_[i,j] = frame[i_,j_]
    end
    imgF = mosaic([imgL, frame_]; nrow=1)
    push!(framestack, imgF)
  end

  # typeof(framestack[1])
  framestack_ = Vector{Matrix{T}}(undef, length(framestack));
  for (i,fr) in enumerate(framestack)
    framestack_[i] = clamp(fr, 0,1);
  end

  return framestack_
end


##



lbls = sortDFG(ls(fg))

# imgL = nothing    # Matrix{RGB{Float64}}()
global img_wPC = nothing
global imgL = nothing


framestack = buildPoseImages(fg, :x0 );


encoder_options = (crf=23, preset="medium")
framerate=90
open_video_out("/tmp/caesar/philos/mosaic.mp4", framestack[1], framerate=framerate, encoder_options=encoder_options) do writer

  for (i,lb) in enumerate(lbls[1:21])
    println("doing all frames for ", lb)

    framestack = buildPoseImages(fg, lb )

    for frame in framestack
      # frame_ .= T.(frame)
      write(writer, frame)
    end

  end # for

end # video


##

# _framestack = framestack[755:756]

##
# frame_ = Matrix{T}(undef, size(framestack_[1])...)

# frame_ = T.(framestack_[1])



##

# movie sequence in world frame


img_wPC = RGB.(img_wPC_[1])
for gim in img_wPC_[2:end]
  # green overlay of latest
  imgG = (px->RGB(0,10*px,0)).(gim);
  imgL = img_wPC + imgG
end

imgL |> imshow

##======================================================================================
## extract camera image sequence


IMGS = []

for lb in lbls
  ents = listDataEntrySequence(fg, lb, r"IMG_CENTER_", sortDFG)
  
  imgs_lb = fetchDataImage.(fg, lb, ents)
  IMGS = [IMGS; imgs_lb]
end

IMGS_ = Vector{typeof(IMGS[1])}(undef, length(IMGS));
IMGS_ .= IMGS;

##

R_ = _Rot.RotMatrix(0.017*pi)
T = eltype(IMGS_[1])

IMGS_R = map(img->warp(img, ImageTransformations.recenter(R_, center(img))), IMGS_);

##

framestack = IMGS_R
frame_ = Matrix{T}(undef, size(IMGS_R[1])...)

encoder_options = (crf=23, preset="medium")
framerate=30
open_video_out("/tmp/caesar/philos/imgs_.mp4", framestack[1], framerate=framerate, encoder_options=encoder_options) do writer
    for frame in framestack
        ax, ay = axes(frame)
        I = ax.parent .+ ax.offset
        J = ay.parent .+ ay.offset
        for (i,i_) in enumerate(I), (j,j_) in enumerate(J)
          frame_[i,j] = frame[i_,j_]
        end
        write(writer, frame_)
    end
end

##


#