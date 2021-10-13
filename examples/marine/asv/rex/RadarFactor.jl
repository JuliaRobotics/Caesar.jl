

using TensorCast
using Manifolds

import IncrementalInference: getSample
import Base: convert, show

import DistributedFactorGraphs: getManifold

export AlignRadarPose2, PackedAlignRadarPose2

"""
$TYPEDEF

This is but one incarnation for how radar alignment factor could work, treat it as a starting point.

Notes
- Stanard `cvt` argument is lambda function to convert incoming images to user convention of image axes,
  - default `cvt` flips image rows so that Pose2 xy-axes corresponds to img[x,y] -- i.e. rows down and across from top left corner.
- Use rescale to resize the incoming images for lower resolution (faster) correlations
- Both images passed to the construct must have the same type some matrix of type `T`.

Example
-------
```julia
arp2 = AlignRadarPose2(img1, img2, 2) # e.g. 2 meters/pixel 
```

See also: [`overlayScanMatcher`](@ref)
"""
struct AlignRadarPose2{T} <: IIF.AbstractManifoldMinimize
  """ reference image for scan matching. """
  im1::Matrix{T}
  """ test image to scan match against the reference image. """
  im2::Matrix{T}
  """ Common grid scale for both images -- i.e. units/pixel.  
  Constructor uses two arguments `gridlength`*`rescale=1`=`gridscale`.
  Arg 0 < `rescale` ≤ 1 is also used to rescale the images to lower resolution for speed. """
  gridscale::Float64

  # replace inner constructor with transform on image
  AlignRadarPose2{T}( im1::AbstractMatrix{T}, 
                      im2::AbstractMatrix{T},
                      gridlength::Real,
                      rescale::Real=1,
                      cvt = (im)->reverse(imresize(im,trunc.(Int, rescale.*size(im))),dims=1)
                    ) where {T} = new{T}( cvt(im1), 
                                          cvt(im2),
                                          rescale*gridlength )
end

AlignRadarPose2(im1::AbstractMatrix{T}, im2::AbstractMatrix{T}, w...) where T = AlignRadarPose2{T}(im1,im2, w...)

getManifold(::IIF.InstanceType{<:AlignRadarPose2}) = getManifold(Pose2Pose2)

getSample( cf::CalcFactor{<:AlignRadarPose2} ) = nothing

function (cf::CalcFactor{<:AlignRadarPose2})(X, p, q)
  M = getManifold(Pose2)
  arp = cf.factor
  tf = Manifolds.compose(M, inv(M, p), q) # for groups
  return evaluateTransform(arp.im1, arp.im2, tf, arp.rescale/arp.gridlength)
end

function Base.show(io::IO, arp::AlignRadarPose2{T}) where {T}
  printstyled(io, "AlignRadarPose2{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    T = ", color=:magenta)
  println(io, T)
  printstyled(io, " }", color=:blue, bold=true)
  println(io)
  println(io, "  size(.im1):      ", size(arp.im1))
  println(io, "    min/max:       ", round(minimum(arp.im1),digits=3), "/", round(maximum(arp.im1),digits=3))
  println(io, "  size(.im2):      ", size(arp.im2))
  println(io, "    min/max:       ", round(minimum(arp.im2),digits=3), "/", round(maximum(arp.im2),digits=3))
  println(io, "  gridscale:       ", arp.gridscale)
  nothing
end

Base.show(io::IO, ::MIME"text/plain", arp::AlignRadarPose2) = show(io, arp)
Base.show(io::IO, ::MIME"application/juno.inline", arp::AlignRadarPose2) = show(io, arp)

## =========================================================================================
## Factor serialization below
## =========================================================================================

struct PackedAlignRadarPose2 <: PackedInferenceType
  im1::Vector{Vector{Float64}}
  im2::Vector{Vector{Float64}}
  gridscale::Float64
end

function convert(::Type{<:PackedAlignRadarPose2}, arp2::AlignRadarPose2)
  TensorCast.@cast pim1[row][col] := arp2.im1[row,col]
  TensorCast.@cast pim1[row] := collect(pim1[row])
  TensorCast.@cast pim2[row][col] := arp2.im2[row,col]
  TensorCast.@cast pim2[row] := collect(pim2[row])
  PackedAlignRadarPose2(
    pim1,
    pim2,
    arp.gridscale )
end

function convert(::Type{<:AlignRadarPose2}, parp2::PackedAlignRadarPose2)
  TensorCast.@cast im1[row,col] := parp2.im1[row][col]
  TensorCast.@cast im2[row,col] := parp2.im2[row][col]
  AlignRadarPose2(
    collect(im1),
    collect(im2),
    parp2.gridscale )
end

#