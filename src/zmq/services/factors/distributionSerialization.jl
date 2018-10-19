import Base.convert

"""
Converter: Packed_MvNormal -> MvNormal
"""
function convert(::Type{Distributions.MvNormal}, pv::Dict)
    len = length(Float64.(pv["mean"]))
    mat = reshape(Float64.(pv["cov"]), len, len)
    return Distributions.MvNormal(Float64.(pv["mean"]), mat)
end

"""
Converter: MvNormal -> Packed_MvNormal
"""
function convert(::Type{Packed_MvNormal}, mvNormal::Distributions.MvNormal)
    v = mvNormal.Σ.mat[:]
    return Packed_MvNormal(mvNormal.μ, v, "MvNormal")
end

"""
Converter: Packed_AliasingScalarSampler -> AliasingScalarSampler
"""
function convert(::Type{IncrementalInference.AliasingScalarSampler}, pv::Dict)
    sampler = IncrementalInference.AliasingScalarSampler(Float64.(pv["samples"]), Float64.(pv["weights"]); SNRfloor=pv["quantile"])
    return sampler
end

"""
Converter: AliasingScalarSampler -> Packed_AliasingScalarSampler
"""
function convert(::Type{Packed_AliasingScalarSampler}, sampler::IncrementalInference.AliasingScalarSampler)
    packed = Packed_AliasingScalarSampler(sampler.domain, sampler.weights.values, 0.0, "AliasingScalarSampler")
    return packed
end
