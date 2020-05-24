# Defining New Variables and Factors

TODO: Smooth text and flow.

## Quick Example in One Dimension

> **Note** these factors already exists in `IncrementalInference` (and many more in `RoME`) and are presented here as a first introduction into the process of defining your own factors.

User scope `Prior`, `LinearOffset`, and `MultiModalOffset` with arbitrary distributions are defined as:
```julia
# import needed to overload IIF.getSample
import IncrementalInference: getSample

struct Prior{T} <: IncrementalInference.FunctorSingleton where T <: SamplableBelief
  z::T
end

#helper function
Prior(t::T) where {T <: SamplableBelief} = Prior{T}(t)

# sampling function
getSample(s::Prior, N::Int=1) = (reshape(rand(s.z,N),1,:), )

struct LinearOffset{T} <: IncrementalInference.FunctorPairwise where T <: SamplableBelief
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

!!! note

    A specialSampler framework is available as experimental.  Please get in touch via a issue or the Caesar.jl slack if you need more information.

## Standardized Serialization

To take advantage of features like `DFG.saveDFG` and `DFG.loadDFG` a user specified type should be able to serialize via JSON standards.  The decision was taken to require bespoke factor types to always be converted into a JSON friendly `struct` which must be prefixed as type name with `PackedPrior{T}`.   Similarly, the user must also overload `Base.convert` as follows -- and note for simplicity we will pack the distribution type here as a string (although being quite hacky):

```julia
# necessary for overloading Base.convert
import Base: convert

struct PackedPrior <: PackedInferenceType
  z::String
end

convert(::Type{PackedPrior}, pr::Prior{<:SamplableBelief}) = PackedPrior(string(pr.z))
convert(::Type{Prior{T}}, pr::PackedPrior) where {T <: SamplableBelief} = Prior{T}(IIF.extractdistribution(pr.z))
```

Now you should be able to `saveDFG` and `loadDFG` your own factor graph types to Caesar.jl / FileDFG standard `.tar.gz` format.

```julia
fg = initfg()
addVariable!(fg, :x0, ContinuousScalar)
addFactor!(fg, [:x0], Prior(Normal()))

# generate /tmp/myfg.tar.gz
saveDFG("/tmp/myfg", fg)

# test loading the .tar.gz (extension optional)
fg2 = initfg()
loadDFG("/tmp/myfg", fg2)

# list the contents
ls(fg2), lsf(fg2)
# should see :x0 and :x0f1 listed
```
