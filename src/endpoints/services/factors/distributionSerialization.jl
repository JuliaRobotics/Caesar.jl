
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
