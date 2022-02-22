import Base.convert


# NEW WITH DFG v0.18
# function convert(::Type{Dict{String, Any}}, dist::PackedSamplableBelief)
#     JSON.parse(JSON.json(dist))
# end

# dictkeys(d::Dict) = (collect(Symbol.(keys(d)))...,)
# dictvalues(d::Dict) = (collect(values(d))...,)

# _namedtuple(d::Dict{<:AbstractString,T}) where {T} =
#     NamedTuple{dictkeys(d)}(dictvalues(d))

# function convert(::Type{<:PackedSamplableBelief}, dict::Dict{String, Any})
#     nt = _namedtuple(dict)
#     convert(PackedSamplableBelief, nt)
# end



"""
Converter: Packed_MvNormal -> MvNormal
"""
function convert(::Type{Distributions.MvNormal}, pv::Dict{String, Any})
    len = length(Float64.(pv["mean"]))
    mat = reshape(Float64.(pv["cov"]), len, len)
    return Distributions.MvNormal(Float64.(pv["mean"]), mat)
end

"""
Converter: MvNormal -> Packed_MvNormal
"""
function convert(::Type{Dict{String, Any}}, mvNormal::Distributions.MvNormal)
    v = mvNormal.Σ.mat[:]
    return JSON.parse(JSON.json(Packed_MvNormal(mvNormal.μ, v, "MvNormal")))
end

"""
Converter: Packed_Normal -> Normal
"""
function convert(::Type{Distributions.Normal}, pv::Dict{String, Any})
    return Distributions.Normal(Float64(pv["mean"]), Float64(pv["std"]))
end

"""
Converter: Normal -> Packed_Normal
"""
function convert(::Type{Dict{String, Any}}, normal::Distributions.Normal)
    return JSON.parse(JSON.json(Packed_Normal(normal.μ, normal.σ, "Normal")))
end


"""
Converter: Packed_AliasingScalarSampler -> AliasingScalarSampler
"""
function convert(::Type{IncrementalInference.AliasingScalarSampler}, pv::Dict{String, Any})
    sampler = IncrementalInference.AliasingScalarSampler(Float64.(pv["samples"]), Float64.(pv["weights"]); SNRfloor=pv["quantile"])
    return sampler
end

"""
Converter: AliasingScalarSampler -> Packed_AliasingScalarSampler
"""
function convert(::Type{Packed_AliasingScalarSampler}, sampler::IncrementalInference.AliasingScalarSampler)
    packed = Packed_AliasingScalarSampler(sampler.domain, sampler.weights.values, 0.0, "AliasingScalarSampler")
    return JSON.parse(JSON.json(packed))
end
