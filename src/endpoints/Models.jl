#
# Convention:
# within Julia? use `convert`
# dict to Julia? use `unmarshal`
# no UTF-8 in packed types?

using JSON, Unmarshal
using Distributions
using IncrementalInference

import Unmarshal: unmarshal
import Base: convert


export
  Packed_MvNormal,
  Packed_SampleWeights


# mutable struct Packed_Distribution
#
# end



## Also do Distributions
# Normal
# Uniform
# Rayleigh
# Categorical
# BallTreeDensity
# AliasingScalarSampler

## Factors
# IIF.Prior
# PriorPose3
# PriorPoint3
# Pose3Pose3
# Prior3ZPR
# Pose3Pose3XYH
# Sonar.jl:
# - RAE
# - Pose3Pose3Offset

# Variables
# Point3
# Pose3
# LHCRawData
# JWSTImage

# C++:
# dist["distType"] =  "MvNormal"
# dist["dist"] = d.ToJson() # specialiezed by inherited distribution
mutable struct Packed_Distribution
  distType::String # MvNormal, Categorical, Rayleigh,
  dist::Dict
end

# "
#     bearing: {
#         distType: MvNormal,
#         dist: {
#             mean,
#             cov
#         }
#     }
# "

# mutable struct Packed_Uniform
#
# end

mutable struct Packed_MvNormal
  mean::Vector{Float64}
  cov::Vector{Float64}
  distType::String ## ?????
end

mutable struct Packed_BearingRange
  bearing::Any
  range::Any
  factorType::String
end

struct BearingRange
  bearing::Distributions.MvNormal
  range::Distributions.MvNormal
end


# bearing = Packed_MvNormal([1, 1, 1], [[1, 1, 1], [1, 1, 1], [1, 1, 1]]  )
# range = Packed_MvNormal([1, 1, 1], [[1, 1, 1], [1, 1, 1], [1, 1, 1]]  )
#
# aha = JSON.json(rangeType)
# test = "{\"mean\":[1.0,1.0,1.0],\"cov\":[[1.0,1.0,1.0],[1.0,1.0,1.0],[1.0,1.0,1.0]]}"


# function convert(::Type{Distributions.MvNormal}, pv::Packed_MvNormal)
#     return Distributions.MvNormal(pv.mean, pv.cov)
# end

### FOR int i = 1 : everything in Distributions... yeesh

function convert(::Type{Packed_MvNormal}, mvNormal::Distributions.MvNormal)
  v = mvNormal.Σ.mat[:]
  return Packed_MvNormal(mvNormal.μ, v, "MvNormal")
end

function convert(::Type{Distributions.MvNormal}, pv::Dict)
  len = length(pv["mean"])
  mat = reshape(Float64.(pv["cov"]), len, len)
  return Distributions.MvNormal(Float64.(pv["mean"]), mat, "MvNormal")
end

function evalType(pt::String)
  getfield(Main, Symbol(pt))
end

function convert(::Type{Prior}, pv::Dict)
    @show pv
    return Prior(MvNormal)
end

function convert(::Type{BearingRange}, pv::Dict)
    @show bearing = convert(evalType(pv["bearing"]["distType"]), pv["bearing"])
    # bearing = convert(eval(parse(pv["bearingType"])), pv["bearing"])
    range = convert(evalType(pv["range"]["distType"]), pv["range"])
    # range = convert(eval(parse(pv["rangeType"])), pv["range"])
    return BearingRange(bearing, range)
end

t = Distributions.MvNormal([5; 5; 5], eye(3))
pt = convert(Packed_MvNormal, t)

packBr = Packed_BearingRange(pt, pt, string(BearingRange))
j = JSON.json(packBr)

# Unpacking here...
jsonTest = "{\"bearing\":{\"mean\":[5.0,5.0,5.0],\"cov\":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],\"distType\":\"MvNormal\"},\"range\":{\"mean\":[5.0,5.0,5.0],\"cov\":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],\"distType\":\"MvNormal\"},\"factorType\":\"BearingRange\"}"
other = JSON.parse(jsonTest)
fulltype = convert(evalType(other["factorType"]), other) # convert





evalType(other["bearingType"])


evalPackedType()

Unmarshal.unmarshal(Distributions.MvNormal, other["range"])


function convert(::Type{Packed_AliasingScalarSampler}, sampleweights::IIF.AliasingScalarSampler) # StatsBase.ProbabilityWeights

end

function convert(::Type{AliasingScalarSampler}, psw::Packed_AliasingScalarSampler)

end

# http://juliastats.github.io/StatsBase.jl/stable/weights.html
mutable struct Packed_AliasingScalarSampler
  samples::Vector{Float64}
  weights::Vector{Float64}
end

## avert your eyes, C++ below
# class MvNormal : public Distribution {
#   std::vector<double> mean_;
#   std::vector<std::vector<double>> cov_;
#
# public:
#   MvNormal(const std::vector<double> &mean,
#            const std::vector<std::vector<double>> &cov)
#       : mean_(mean), cov_(cov) {}
#   json ToJson(void) const {
#     json j;
#     j["mean"] = mean_;
#     j["cov"] = cov_;
#     return (j);
#   }
# };
