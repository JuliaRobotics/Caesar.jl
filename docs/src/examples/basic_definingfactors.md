# Defining New Variables and Factors

TODO: Smooth text and flow.

## Quick Example in One Dimension

> **Note** these factors already exists in `IncrementalInference` (and many more in `RoME`) and are presented here as a first introduction into the process of defining your own factors.

User scope `Prior`, `LinearOffset`, and `MultiModalOffset` with arbitrary distributions are defined as:
```julia
import IncrementalInference: getSample

struct Prior{T} <: IncrementalInference.FunctorSingleton where T <: Distribution
  z::T
end
getSample(s::Prior, N::Int=1) = (reshape(rand(s.z,N),1,:), )
struct LinearOffset{T} <: IncrementalInference.FunctorPairwise where T <: Distribution
  z::T
end
getSample(s::LinearOffset, N::Int=1) = (reshape(rand(s.z,N),1,:), )
function (s::LinearOffset)(res::Array{Float64},
                           userdata::FactorMetadata,
                           idx::Int,
                           meas::Tuple{Array{Float64, 2}},
                           X1::Array{Float64,2},
                           X2::Array{Float64,2}  )
  #
  res[1] = meas[1][idx] - (X2[1,idx] - X1[1,idx])
  nothing
end
struct MultiModalOffset <: IncrementalInference.FunctorPairwise
  z::Vector{Distribution}
  c::Categorical
end
getSample(s::MultiModalOffset, N::Int=1) = (reshape.(rand.(s.z, N),1,:)..., rand(s.c, N))
function (s::MultiModalOffset)(res::Array{Float64},
                               userdata::FactorMetadata,
                               idx::Int,
                               meas::Tuple,
                               X1::Array{Float64,2},
                               X2::Array{Float64,2}  )
  #
  res[1] = meas[meas[end][idx]][idx] - (X2[1,idx] - X1[1,idx])
  nothing
end
```
Notice the residual function relating to the two `PairwiseFunctor` derived definitions.
The one dimensional residual functions, `res[1] = measurement - prediction`, are used during inference to approximate the convolution of conditional beliefs from the sample approximate marginal beliefs of the connected variables.
