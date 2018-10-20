using Distributions
using IncrementalInference

import Unmarshal: unmarshal
import Base: convert

"""
Converter: Prior -> JSON-Compliant Prior
"""
function convert(::Type{Dict{String, Any}}, prior::IncrementalInference.Prior)
    # Genericize to any packed type next.
    z =convert(Packed_MvNormal, prior.Z)
    return Packed_PriorPrior(z)
end

"""
Converter: PackedPrior::Dict{String, Any} -> Prior
"""
function convert(::Type{Prior}, prior::Dict{String, Any})
    # Genericize to any packed type next.
    z = prior["measurement"][1]
    z = convert(_evalType(z["distType"]), z)
    return Prior(z)
end

"""
Converter: Pose2Pose2::Dict{String, Any} -> Pose2Pose2
"""
function convert(::Type{Pose2Pose2}, prior::Dict{String, Any})
    # Genericize to any packed type next.
    z = prior["measurement"][1]
    z = convert(_evalType(z["distType"]), z)
    return Pose2Pose2(z)
end

# """
# Converter: Pose2Pose2::Dict{String, Any} -> Pose2Pose2
# """
# function convert(::Type{Pose2Pose2}, prior::Dict{String, Any})
#     # Genericize to any packed type next.
#     z = prior["measurement"][1]
#     z = convert(_evalType(z["distType"]), z)
#     return Prior(z)
# end
