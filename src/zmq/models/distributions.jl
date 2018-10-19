export
    Packed_MvNormal,
    Packed_AliasingScalarSampler

mutable struct Packed_MvNormal
  mean::Vector{Float64}
  cov::Vector{Float64}
  distType::String
end

mutable struct Packed_AliasingScalarSampler
  samples::Vector{Float64}
  weights::Vector{Float64}
  snrFloor::Nullable{Float64}
  distType::String # AliasingScalarSampler
end
