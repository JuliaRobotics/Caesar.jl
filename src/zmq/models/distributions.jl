export
    Packed_MvNormal

mutable struct Packed_MvNormal
  mean::Vector{Float64}
  cov::Vector{Float64}
  distType::String
end
