

using TensorCast
using Manifolds

import IncrementalInference: getSample
import Base: convert

import DistributedFactorGraphs: getManifold

"""
$TYPEDEF

This is but one incarnation for how radar alignment factor could work, treat it as a starting point.

Notes
- Stanard `cvt` argument is lambda function to convert incoming images to user convention of image axes,
  - default `cvt` flips image rows so that Pose2 xy-axes corresponds to img[x,y] -- i.e. rows down and across from top left corner.
- Use rescale to resize the incoming images for lower resolution (faster) correlations

Example
-------
```julia
using LinearAlgebra
arp2 = AlignRadarPose2(sweep[10], sweep[11], MvNormal(zeros(3), diagm([5;5;pi/4])))
```
"""
struct AlignRadarPose2{T} <: IIF.AbstractManifoldMinimize
  im1::Matrix{T}
  im2::Matrix{T}
  gridlength::Float64
  rescale::Float64

  # replace inner constructor with transform on image
  AlignRadarPose2{T}( im1::AbstractMatrix{T}, 
                      im2::AbstractMatrix{T},
                      gridlength::Real,
                      rescale::Real=1,
                      cvt = (im)->reverse(imresize(im,trunc.(Int, rescale.*size(im))),dims=1)
                    ) where {T} = new{T}( cvt(im1), 
                                          cvt(im2) )
end

AlignRadarPose2(im1::AbstractMatrix{T}, im2::AbstractMatrix{T}, w...) where T = AlignRadarPose2{T}(im1,im2, w...)

#

getManifold(::IIF.InstanceType{<:AlignRadarPose2}) = getManifold(Pose2Pose2)

function getSample( cf::CalcFactor{<:AlignRadarPose2} )
  # M = getManifold(Pose2)
  # e0 = identity_element(M)
  
  # rp2 = cf.factor
  # closure on inner function
  # cost(x::AbstractVector) = evaluateTransform(rp2.im1,rp2.im2, x...)
  # pres = rand(rp2.PreSampler)
  # ignoring failures
  # out = optimize(cost, pres, NelderMead()).minimizer

  # return tangent vector element
  # return hat(M, e0, out)
  return nothing
end

function (cf::CalcFactor{<:AlignRadarPose2})(X, p, q)
  M = getManifold(Pose2)
  # e0 = identity_element(M, p)
  rp2 = cf.factor
  
  tf = Manifolds.compose(M, inv(M, p), q) # for groups
  
  return evaluateTransform(rp2.im1, rp2.im2, tf)
  
  # q̂ = Manifolds.compose(M, p, exp(M, e0, X)) #for groups
  # #TODO allocalte for vee! see Manifolds #412, fix for AD
  # Xc = zeros(3)
  # vee!(M, Xc, q, log(M, q, q̂))
  # return Xc
end

struct PackedAlignRadarPose2 <: PackedInferenceType
  im1::Vector{Vector{Float64}}
  im2::Vector{Vector{Float64}}
  gridlength::Float64
  rescale::Float64

  # PreSampler::String
  # p2p2::PackedPose2Pose2
end

function convert(::Type{<:PackedAlignRadarPose2}, arp2::AlignRadarPose2)
  TensorCast.@cast pim1[row][col] := arp2.im1[row,col]
  TensorCast.@cast pim1[row] := collect(pim1[row])
  TensorCast.@cast pim2[row][col] := arp2.im2[row,col]
  TensorCast.@cast pim2[row] := collect(pim2[row])
  PackedAlignRadarPose2(
    pim1,
    pim2,
    arp2.gridlength,
    arp.rescale )
end

function convert(::Type{<:AlignRadarPose2}, parp2::PackedAlignRadarPose2)
  TensorCast.@cast im1[row,col] := parp2.im1[row][col]
  TensorCast.@cast im2[row,col] := parp2.im2[row][col]
  AlignRadarPose2(
    collect(im1),
    collect(im2),
    parp2.gridlength,
    parp2.rescale )
end
