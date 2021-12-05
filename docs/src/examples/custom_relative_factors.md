# [Custom Relative Factor](@id custom_relative_factor)

| Required                                  | Brief description                                                                      |
|:------------------------------------------|:-------------------------------------------------------------------------------------- |
| `MyFactor`  struct                        | Prior (`<:AbstractPrior`) or Relative (`<:AbstractManifoldMinimize`) factor definition |
| `getManifold`                             | The manifold of the factor |
| `(cfo::CalcFactor{<:MyFactor})`           | Factor residual function |
| **Optional methods**                      | **Brief description**                                                                  |
| `getSample(cfo::CalcFactor{<:MyFactor})`  | Get a sample from the measurement model |

## Define the relative struct

Previously we looked at making a [Custom Prior Factor](@ref custom_prior_factor).  This section describes how to build *relative factors*.  Relative factors introduce relative-only information between variables in the factor graph, and do not add any absolute information.  For example, a rigid transform between two variables is a relative relationship, regardless of their common absolute position in the world.

Lets look at either the [`EuclidDistance`](@ref) of [`Pose2Pose2`](@ref) factors as simple examples.  First, create the uniquely named factor struct:
```julia
struct EuclidDistance{T <: IIF.SamplableBelief} <: IIF.AbstractManifoldMinimize
  Z::T
end
```
New relative factors should either inheret from `<:AbstractManifoldMinimize`, `<:AbstractRelativeMinimize`, or `<:AbstractRelativeRoots`.  These are all subtypes of `<:AbstractRelative`.  There are only two abstract super types, `<:AbstractPrior` and `<:AbstractRelative`.

## Specialized Dispatch (`getManifold`, `getSample`)

Relative factors involve computaton, these computations must be performed on some manifold.  Custom relative factors require that the [`getManifold`](@ref) function be overridded.  Here two examples are given for reference:
```julia
# import override/specialize the multiple dispatch
import DistributedFactorGraphs: getManifold

# two examples of existing functions in the standard libraries
DFG.getManifold(::EuclidDistance) = Manifolds.TranslationGroup(1)
DFG.getManifold(::Pose2Pose2) = Manifolds.SpecialEuclidean(2)
```

Extending the `getSample` method for our `EuclidDistance` factor example is not required, since the default dispatch using field `.Z <: SamplableBelief` will already be able to sample the measurement -- see [Specialized `getSample`](@ref specialized_getSample).

One **important note** is that `getSample` for `<:AbstractRelative` factors should return measurement values as manifold tangent vectors -- for computational efficiency reasons.

If more advanced sampling is required, extend the `getSample` function. 

```julia
function getSample(cf::CalcFactor{<:Pose2Pose2}) 
  M = getManifold(cf.factor)
  ϵ = getPointIdentity(Pose2)
  X = sampleTangent(M, cf.factor.Z, ϵ)
  return X
end
```

The return type for `getSample` is unrestricted, and will be passed to the residual function "as-is".

!!! note
    Default dispatches in `IncrementalInference` will try use `cf.factor.Z` to `samplePoint` on manifold (for `<:AbstractPrior`) or `sampleTangent` (for `<:AbstractRelative`), which simplifies new factor definitions.  If, however, you wish to build more complicated sampling processes, then simply define your own `getSample(cf::CalcFactor{<:MyFactor})` function.

## [Factor Residual Function](@id factor_residual_function)

The selection of `<:IIF.AbstractManifoldMinimize`, akin to earlier `<:AbstractPrior`, instructs IIF to find the minimum of the provided residual function.  The residual function is used during inference to approximate the convolution of conditional beliefs from the approximate beliefs of the connected variables in the factor graph.  Conceptually, the residual function is usually something akin to `residual = measurement - prediction`, but does not have to follow the exact recipe.

The returned value (the factor measurement) from `getSample` will always be passed as the first argument (e.g. `X`) to the factor residual function.  
```julia
# first residual function example
(cf::CalcFactor{<:EuclidDistance})(X, p, q) = X - norm(p .- q)

# second residual function example
function (cf::CalcFactor{<:Pose2Pose2})(X, p, q)
    M = getManifold(Pose2)
    q̂ = Manifolds.compose(M, p, exp(M, identity_element(M, p), X))
    Xc = vee(M, q, log(M, q, q̂))
    return Xc
end
```

!!! note
    At present (2021) the residual function should return the residual value as a coordinate (not as tangent vectors or manifold points).  Ongoing work is in progress, and likely to return residual values as manifold tangent vectors instead.

It is recommended to leave the incoming types unrestricted.  If you must define the types, make sure to allow sufficient dispatch freedom (i.e. dispatch to concrete types) and not force operations to "non-concrete" types.  Usage can be very case specific, and hence better to let Julia type-inference automation do the hard work of inferring the concrete types.

### Serialization

Serialization of factors is also discussed in more detail at [Standardized Factor Serialization](@ref factor_serialization).
