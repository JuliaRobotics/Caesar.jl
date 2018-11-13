export
    Packed_Normal,
    Packed_MvNormal,
    Packed_AliasingScalarSampler,
    Packed_BallTreeDensity

mutable struct Packed_Normal
  mean::Float64
  std::Float64
  distType::String
end

mutable struct Packed_MvNormal
  mean::Vector{Float64}
  cov::Vector{Float64}
  distType::String
end

mutable struct Packed_AliasingScalarSampler
  samples::Vector{Float64}
  weights::Vector{Float64}
  quantile::Union{Float64, Nothing}
  distType::String # AliasingScalarSampler
end

mutable struct Packed_BallTreeDensity
  dim::Int  # point dimension
  kernelType::String
  bandwidth::Vector{Float64}
  weights::Vector{Float64}
  points::Vector{Float64} # Stack points as a 1D vector.
end
