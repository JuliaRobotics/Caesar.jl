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
using Images, FileIO, Base64

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
  im1::String
  im2::String
  PreSampler::String
  p2p2::PackedPose2Pose2
end

function image2string(image::Array{Float64, 2})::String
  io = IOBuffer()
  save(Stream(format"PNG", io), image)
  return base64encode(io.data)
end

function string2image(stringdata::String)::Array{Float64, 2}
  io = IOBuffer(base64decode(stringdata))
  return load(Stream(format"PNG", io))
end


function convert(::Type{PackedAlignRadarPose2}, arp2::AlignRadarPose2)
  return PackedAlignRadarPose2(
    image2string(arp2.im1),
    image2string(arp2.im2),
    string(arp2.PreSampler),
    convert(PackedPose2Pose2, arp2.p2p2))
end

function convert(::Type{AlignRadarPose2}, parp2::PackedAlignRadarPose2)
  AlignRadarPose2(
    string2image(parp2.im1),
    string2image(parp2.im2),
    extractdistribution(parp2.PreSampler),
    convert(Pose2Pose2, parp2.p2p2))
end
