# [Custom Relative Factor](@id custom_relative_factor)

## One Dimension Example

Previously we looked at adding a prior.  This section demonstrates relative factors on manifold.  These are factors that introduce only relative information between variables in the factor graph.

Lets look at the Pose2Pose2 factor available in RoME as an example.  First, lets create the factor as before 
```julia
struct Pose2Pose2{T <: IIF.SamplableBelief} <: IIF.AbstractManifoldMinimize
  Z::T
end

DFG.getManifold(::Pose2Pose2) = Manifolds.SpecialEuclidean(2)
```

Extending the `getSample` method for our new `Pose2Pose2` factor is optional in this case.
The default behavior for sampling on `<:AbstractManifoldMinimize` factors defined on Group Manifolds is:
- The returned value from `getSample` represents a tangent vector at the identity element.
- The `SamplableBelief` shall be in the field `Z` and that shall be enough to fully define the factor.

If more advanced sampling is required, the `getSample` function can be extended. 

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

The selection of `<:IIF.AbstractManifoldMinimize`, akin to earlier `<:AbstractPrior`, instructs IIF to find the minimum of the provided residual function.
That is the one dimensional residual function, `residual = measurement - prediction`, is used during inference to approximate the convolution of conditional beliefs from the approximate beliefs of the connected variables in the factor graph.

The returned value (the factor measurement) from getSample will always be passed as the first argument (`X`) in the factor calculation.
The residual function should return a coordinate. 
```julia
function (cf::CalcFactor{<:Pose2Pose2})(X, p, q)
    M = getManifold(Pose2)
    q̂ = Manifolds.compose(M, p, exp(M, identity_element(M, p), X))
    Xc = vee(M, q, log(M, q, q̂))
    return Xc
end
```

!!! note
    Measurements and variables passed in to the factor residual function have related but potentially different types during construction or computation.  It is recommended to leave these incoming types unrestricted.  If you must define the types, make sure to allow sufficient dispatch freedom (i.e. dispatch to concrete types) and not force operations to "non-concrete" types.  Usage can be very case specific, and hence better to let Julia type-inference automation do the hard work of inferring the concrete types.

[//]: # (#TODO ### Advanced Sampling)

