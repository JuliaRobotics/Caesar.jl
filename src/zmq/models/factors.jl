export
    PackedPrior

mutable struct Packed_Prior
    measurement::Vector{Dict{String, Any}}
    distType
end
