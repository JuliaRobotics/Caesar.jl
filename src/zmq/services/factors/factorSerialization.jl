using Distributions
using IncrementalInference

import Unmarshal: unmarshal
import Base: convert

"""
Converter: Prior -> Packed_Prior
"""
function convert(::Type{Packed_Prior}, prior::IncrementalInference.Prior)
    # Genericize to any packed type next.
    z =convert(Packed_MvNormal, prior.Z)
    return Packed_PriorPrior(z)
end

"""
Converter: PackedPrior::Dict{String, Any} -> Prior
"""
function convert(::Type{Prior}, prior::Any)
    # Genericize to any packed type next.
    z = prior["measurement"][1]
    z = convert(_evalType(z["distType"]), z)
    return Prior(z)
end

"""
Converter: PackedPose2Pose2::Dict{String, Any} -> Pose2Pose2
"""
function convert(::Type{Pose2Pose2}, prior::Any)
    # Genericize to any packed type next.
    z = prior["measurement"][1]
    z = convert(_evalType(z["distType"]), z)
    return Prior(z)
end
