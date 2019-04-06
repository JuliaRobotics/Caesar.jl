
import Base: convert

export
  MultiplyDistributionsRequest

struct MultiplyDistributionsRequest
    label::String
    variableType::String
    N::Union{Int64, Nothing}
    labels::Vector{String}
end
