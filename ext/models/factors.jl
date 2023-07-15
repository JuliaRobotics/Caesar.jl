



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


##==============================================================================
## LEGACY, towards Sidecar, MUST DEPRECATE
##==============================================================================

"""
Converter: Prior -> PackedPrior::Dict{String, Any}

FIXME see DFG #590 for consolidation with Serialization and Marshaling
"""
function convert(::Type{Dict{String, Any}}, prior::IncrementalInference.Prior)
  @error("Obsolete, use pack/unpack converters instead")
  z = convert(Type{Dict{String, Any}}, prior.Z)
  return Packed_Factor([z], "Prior")
end

"""
Converter: PackedPrior::Dict{String, Any} -> Prior

FIXME see DFG #590 for consolidation on Serialization and Marshaling
"""
function convert(::Type{<:Prior}, prior::Dict{String, Any})
  @error("Obsolete, use pack/unpack converters instead")
  # Genericize to any packed type next.
  z = prior["measurement"][1]
  z = convert(DFG.getTypeFromSerializationModule(z["distType"]), z)
  return Prior(z)
end