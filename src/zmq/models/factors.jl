



export
    Packed_Factor


"""
    $TYPEDEF

FIXME see DFG #590 on Serialization and Marshaling

This type and use must be reviewed as part of the next gen (4Q20) serialization upgrades.

`::Any` has slower performance, so we should try avoid (but not critical)
"""
mutable struct Packed_Factor
    measurement::Vector{Dict{String, Any}}
    distType::String
end
