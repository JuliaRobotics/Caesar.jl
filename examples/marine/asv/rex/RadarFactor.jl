

using TensorCast
using Manifolds

import IncrementalInference: getSample
import Base: convert

import DistributedFactorGraphs: getManifold

"""
$TYPEDEF

This is but one incarnation for how radar alignment factor could work, treat it as a starting point.

Example
-------
```julia
using LinearAlgebra
arp2 = AlignRadarPose2(sweep[10], sweep[11], MvNormal(zeros(3), diagm([5;5;pi/4])))
```

"""
struct AlignRadarPose2{T <: Real, P <: SamplableBelief} <: IIF.AbstractManifoldMinimize
  im1::Matrix{T}
  im2::Matrix{T}
  PreSampler::P
  p2p2::Pose2Pose2
end

AlignRadarPose2(im1::Matrix{T}, im2::Matrix{T}, pres::P) where {T <: Real, P <: SamplableBelief} = AlignRadarPose2{T,P}(im1, im2, pres, Pose2Pose2(pres))

getManifold(::IIF.InstanceType{<:AlignRadarPose2}) = getManifold(Pose2Pose2)

function getSample( cf::CalcFactor{<:AlignRadarPose2} )

  M = getManifold(Pose2)
  e0 = identity_element(M)
  
  rp2 = cf.factor
  # closure on inner function
  error("MOVE TO RESIDUAL FUNCTION ONLY")
  cost(x::AbstractVector) = evaluateTransform(rp2.im1,rp2.im2, x...)
  pres = rand(rp2.PreSampler)
  # ignoring failures
  out = optimize(cost, pres, NelderMead()).minimizer

  # return tangent vector element
  return hat(M, e0, out)
end

function (cf::CalcFactor{<:AlignRadarPose2})(X, p, q)
  @assert X isa ProductRepr "Pose2Pose2 expects measurement sample X to be a Manifolds tangent vector, not coordinate or point representation.  Got X=$X"
  M = getManifold(Pose2)
  e0 = identity_element(M, p)
  q̂ = Manifolds.compose(M, p, exp(M, e0, X)) #for groups
  #TODO allocalte for vee! see Manifolds #412, fix for AD
  Xc = zeros(3)
  vee!(M, Xc, q, log(M, q, q̂))
  return Xc
end

struct PackedAlignRadarPose2 <: PackedInferenceType
  im1::Vector{Vector{Float64}}
  im2::Vector{Vector{Float64}}
  PreSampler::String
  p2p2::PackedPose2Pose2
end

function convert(::Type{<:PackedAlignRadarPose2}, arp2::AlignRadarPose2)
  TensorCast.@cast pim1[row][col] := arp2.im1[row,col]
  TensorCast.@cast pim1[row] := collect(pim1[row])
  TensorCast.@cast pim2[row][col] := arp2.im2[row,col]
  TensorCast.@cast pim2[row] := collect(pim2[row])
  PackedAlignRadarPose2(
    pim1,
    pim2,
    convert(PackedSamplableBelief, arp2.PreSampler),
    convert(PackedPose2Pose2, arp2.p2p2))
end

function convert(::Type{<:AlignRadarPose2}, parp2::PackedAlignRadarPose2)
  TensorCast.@cast im1[row,col] := parp2.im1[row][col]
  TensorCast.@cast im2[row,col] := parp2.im2[row][col]
  AlignRadarPose2(
    collect(im1),
    collect(im2),
    convert(SamplableBelief, parp2.PreSampler), #   extractdistribution(parp2.PreSampler),
    convert(Pose2Pose2, parp2.p2p2))
end
