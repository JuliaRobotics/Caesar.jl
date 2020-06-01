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

using TensorCast

import IncrementalInference: getSample
import Base: convert


struct AlignRadarPose2{T <: Real, P <: SamplableBelief} <: FunctorPairwise
  im1::Matrix{T}
  im2::Matrix{T}
  PreSampler::P
  p2p2::Pose2Pose2
end

AlignRadarPose2(im1::Matrix{T}, im2::Matrix{T}, pres::P) where {T <: Real, P <: SamplableBelief} = AlignRadarPose2{T,P}(im1, im2, pres, Pose2Pose2(pres))

function getSample(rp2::AlignRadarPose2, N::Int=1)

  # closure on inner function
  cost(x::AbstractVector) = evaluateTransform(rp2.im1,rp2.im2,x...)

  pres = rand(rp2.PreSampler, N)
  out = zeros(3, N)
  TASKS = Vector{Task}(undef, N)
  for i in 1:N
    # ignoring failures
      TASKS[i] = Threads.@spawn optimize(cost, pres[:, $i], NelderMead()).minimizer
    # out[:,i] = optimize(cost, pres[:,i], NelderMead()).minimizer
  end
    # retrieve threaded results
    @sync for i in 1:N
      @async out[:,$i] .= fetch(TASKS[$i])
    end

  # only using out, but return pres if user wants to look at it.
  return (out, pres)
end

function (rp2::AlignRadarPose2)(res::Vector{Float64},
                           userdata::FactorMetadata,
                           idx::Int,
                           meas::Tuple,
                           wXi::Array{Float64,2},
                           wXj::Array{Float64,2})
  #
  rp2.p2p2(res, userdata, idx, meas, wXi, wXj)
  res
end


struct PackedAlignRadarPose2 <: PackedInferenceType
  im1::Vector{Float64}
  im2::Vector{Float64}
  PreSampler::String
  p2p2::PackedPose2Pose2
end

function convert(::Type{PackedAlignRadarPose2}, arp2::AlignRadarPose2)
  TensorCast.@cast pim1[row][col] := arp2.im1[row,col]
  TensorCast.@cast pim2[row][col] := arp2.im2[row,col]
  PackedAlignedRadarPose2(pim1, pim2, string(arp2.PreSampler), convert(PackedPose2Pose2, arp2.p2p2))
end
function convert(::Type{AlignRadarPose2}, parp2::PackedAlignRadarPose2)
  TensorCast.@cast im1[row,col] := parp2.pim1[row][col]
  TensorCast.@cast im2[row,col] := parp2.pim2[row][col]
  AlignedRadarPose2(im1, im2, extractdistribution(arp2.PreSampler), convert(Pose2Pose2, parp2.p2p2))
end
