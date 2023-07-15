module CaesarImagesExt

@info "Loading Caesar tools related to Images.jl."


using Images
# using ImageTransformations
using ColorVectorSpace
using UUIDs
using TensorCast
using StaticArrays
using Manifolds
using DocStringExtensions
using ProgressMeter
using Optim

using Caesar # TODO try reduce to just import Caesar ... below
import Caesar._PCL as _PCL

# import DistributedFactorGraphs: getManifold

import Base: convert, show

import GeometricalPredicates as GeoPr

import ApproxManifoldProducts: sample, _update!
import IncrementalInference: getSample, preambleCache, _update!, getManifold

import Caesar: applyMaskImage, makeMaskImage, makeMaskImages, imhcatPretty, toImage
import Caesar: writevideo, csmAnimationJoinImgs, csmAnimateSideBySide, makeVideoFromData
import Caesar: overlayScanMatcher
import Caesar: ScanMatcherPose2, PackedScanMatcherPose2
import Caesar: _PARCHABLE_PACKED_CLOUD
import Caesar: ScatterAlign, ScatterAlignPose2, ScatterAlignPose3
import Caesar: PackedScatterAlignPose2, PackedScatterAlignPose3

"""
    $SIGNATURES

Also see: [`toROSImage`](@ref)
"""
function toImage(msgd::Dict{String,Any})
  data = haskey(msgd, "data_b64") ? base64decode(msgd["data_b64"]) : UInt8.(msgd["data"])
  h, w = msgd["height"], msgd["width"]
  @show h, w, msgd["step"]
  if msgd["encoding"] == "mono8" || msgd["encoding"] == "8UC1"
    # img_ = normedview(N0f8, data)
    # reshape(img_, w, h)'
    img = Matrix{Gray{N0f8}}(undef, h, w)
    # assuming one endian type for now, TODO both little and big endian
    for i in 1:h, j in 1:w
      img[i,j] = Gray{N0f8}(data[msgd["step"]*(i-1)+j]/(2^8 - 1))
    end
    img
  elseif msgd["encoding"] == "mono16" || msgd["encoding"] == "16UC1"
    img_ = normedview(N0f16, data)
    reshape(img_, w, h)'
  else
    error("Conversion for ROS sensor_msgs.Image encoding not implemented yet $(msgd["encoding"])")
  end
end

function makeMaskImage(
  img::AbstractMatrix, 
  nodes::AbstractVector = [
    (180.0, 0.0);  # ll
    (10.0, 600.0); # lr
    (0.0, 600.0);  # ur
    (0.0, 0.0);    # ul
  ],
)
  h,w = size(img)
  # h,w = 400,640
  buffer = zeros(UInt8, h, w)
  
  points = []
  for nd in nodes
    push!(points, GeoPr.Point(nd...))
  end
  # (ll,lr,ur,ul)
  poly = GeoPr.Polygon(points...)
  
  # populate buffer
  for x in collect(1:1:h), y in collect(1:1:w)
    if GeoPr.inpolygon(poly, GeoPr.Point(x, y)) 
      buffer[x, y] = UInt8(1) 
    end 
  end

  return buffer, poly
end


"""
    $SIGNATURES


```julia
jstr_Mask_BOT = \"""
[
  [[180.0,0.0],[10.0,600.0],[0.0,600.0],[0.0,0.0]],
  [[400.0,0.0],[400.0,640.0],[310.0,640.0],[315.0,165.0],[360.0,0.0],[399.0,0.0]]
]
\"""

mnodes = JSON3.read(jstr_Mask_BOT)
mask_u8, polys = makeMaskImages(img, mnodes)
```
"""
function makeMaskImages(
  img::AbstractMatrix,
  mnodes::AbstractVector{<:AbstractVector}
)
  mask = zeros(UInt8, size(img)...)
  polys = []

  for nodes in mnodes
    @show nodes
    mk, poly = makeMaskImage(img, nodes)
    mask += mk
    push!(polys, poly)
  end

  return mask, polys
end


"""
    $SIGNATURES

Apply color to masked region onto a return copy of the image.
"""
function applyMaskImage(
  img::AbstractMatrix{<:Colorant},
  mask::AbstractMatrix,
  color::Colorant = RGB{N0f8}(0.0,0.4,0.4)
)
  imask = mask.*-1 .+ 1
  suppr = RGB.(mask)  .⊙ img
  keepr = RGB.(imask) .⊙ img
  color .⊙ suppr + keepr 
end



function imhcatPretty(iml::AbstractMatrix{<:Colorant},
                      imr::AbstractMatrix{<:Colorant} )
  #
  imll = similar(iml)
  fill!(imll, RGB{N0f8}(1,1,1))

  # where to place imr
  heightratio, widthratio = size(imll,1)/size(imr,1), size(imll,2)/size(imr,2)   
  minratio = 0.9*minimum([heightratio, widthratio])
  imrr = Images.imresize(imr, ratio=minratio)
  offsr = round.(Int, 0.05*[size(imll)...])
  endir = [size(imrr)...] + offsr
  imll[offsr[1]:endir[1]-1,offsr[2]:endir[2]-1] .= imrr

  hcat(iml,imll)
end

include("Images/ImageToVideoUtils.jl")
include("Images/ScanMatcherUtils.jl")
include("Images/ScanMatcherPose2.jl")
include("Images/ScatterAlignPose2.jl")


end # module