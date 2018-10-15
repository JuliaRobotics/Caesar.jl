
"""
Converter: Packed -> MvNormal
"""
function convert(::Type{Distributions.MvNormal}, pv::Dict)
  @show len = length(pv["mean"])
  @show mat = reshape(Float64.(pv["cov"]), len, len)
  @show pv["mean"]
  return Distributions.MvNormal(Float64.(pv["mean"]), mat)
end

# function convert(::Type{Distributions.MvNormal}, pv::Dict)
#   len = length(pv["mean"])
#   mat = reshape(Float64.(pv["cov"]), len, len)
#   return Distributions.MvNormal(Float64.(pv["mean"]), mat, "MvNormal")
# end
