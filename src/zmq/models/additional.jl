
import Base: convert

export
  MultiplyDistributionsRequest

struct MultiplyDistributionsRequest
    weights::Vector{Packed_BallTreeDensity}
end
