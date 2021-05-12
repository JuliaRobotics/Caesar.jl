
# Parametric Solve (Experimental)

Note that parametric solve (i.e. conventional Gaussians) is currently supported as an experimental feature which might appear more buggy.  Familiar parametric methods should become fully integrated and we invite comments or contributions from the community.  A great deal of effort has gone into finding the best abstractions to support multiple factor graph solving strategies.

## Batch Parametric

```@docs
solveGraphParametric
IncrementalInference.solveGraphParametric!
```

Initializing the parametric solve from existing values can be done with the help of:

```@docs
initParametricFrom!
```

## [Defining Factors to Support a Parametric Solution (Experimental)](@id parametric_factors)

Factor that supports a parametric solution, with supported distributions (such as `Normal` and `MvNormal`), can be used in a parametric batch solver `solveGraphParametric`. 


### `getParametricMeasurement`

Parameteric calculations require the mean and covariance from Gaussian measurement functions (factors) using the function

```@docs
IncrementalInference.getParametricMeasurement
```

`getParametricMeasurement` defaults to looking for a supported distribution in field `.Z` followed by `.z`. Therefore, if the factor uses this fieldname, `getParametricMeasurement` does not need to be extended.  You can extend by simply implementing, for example, your own `IncrementalInference.getParametricMeasurement(f::OtherFactor) = m.density`.

For this example, the `Z` field will automatically be detected used by default for `MyFactor` from above.

```julia
struct MyFactor{T <: SamplableBelief} <: IIF.AbstractRelativeRoots
  Z::T
end
```

An example of where implementing `getParametricMeasurement` is needed can be found in the RoME factor [`Pose2Point2BearingRange`](@ref)
```julia
import getParametricMeasurement
function getParametricMeasurement(s::Pose2Point2BearingRange{<:Normal, <:Normal})

  meas = [mean(s.bearing), mean(s.range)]
  iΣ = [1/var(s.bearing)             0;
                      0  1/var(s.range)]

  return meas, iΣ
end
```

### The Factor
The factor is evaluated in a cost function using the [Mahalanobis distance](https://en.wikipedia.org/wiki/Mahalanobis_distance) and the measurement should therefore match the residual returned.  

### Optimization
[`IncrementalInference.solveGraphParametric!`](@ref) uses Optim.jl. The factors that are supported should have a gradient and Hessian available/exists and therefore it makes use of `TwiceDifferentiable`. Full control of Optim's setup is possible with keyword arguments.  


