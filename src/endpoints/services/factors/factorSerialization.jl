using Distributions
using IncrementalInference

import Unmarshal: unmarshal
import Base: convert

function _evalType(pt::String)::Type
    try
        getfield(Main, Symbol(pt))
    catch ex
        io = IOBuffer()
        showerror(io, ex, catch_backtrace())
        err = String(take!(io))
        error("_evalType: Unable to locate factor type '$pt' in main context. Please check that this factor type is loaded into main. Stack trace = $err")
    end
end

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
    z=convert(evalType(pv["bearing"]["distType"]), prior.Z)
    return Prior(z)
end

# """
# Converter: Packed_BearingRange -> BearingRange
# """
# function convert(::Type{Pose2Point2BearingRange}, pv::Dict)
#     @show bearing = convert(evalType(pv["bearing"]["distType"]), pv["bearing"])
#     # bearing = convert(eval(parse(pv["bearingType"])), pv["bearing"])
#     range = convert(evalType(pv["range"]["distType"]), pv["range"])
#     # range = convert(eval(parse(pv["rangeType"])), pv["range"])
#     return Pose2Point2BearingRange(bearing, range)
# end
