
# Parametric Solve (Experimental)

## Batch 

```@docs
solveFactorGraphParametric
IncrementalInference.solveFactorGraphParametric!
```

## [Defining Factors to Support a Parametric Solution (Experimental)](@id parametric_factors)

Factor that supports a parametric solution, with supported distributions (such as `Normal` and `MvNormal`), can be used in a parametric batch solver `solveFactorGraphParametric`. 


### `getParametricMeasurement`

```@docs
IncrementalInference.getParametricMeasurement
```

`getParametricMeasurement` defaults to looking for a supported distribution in field `.Z` followed by `.z`. Therefore, if the factor uses this fieldname, `getParametricMeasurement` does not need to be extended.

For example the `Z` field will be used by default in `MyFactor` from above.

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
The factor is evaluated in a cost function using the Mahalanobis distance and the measurement should therefore match the residual returned.  

### Optimization
[`solveFactorGraphParametric`](@ref) uses Optim.jl. The factors that are supported should have a gradient and Hessian available/exists and therefore it makes use of `TwiceDifferentiable`. Full control of Optim's setup is possible with keyword arguments.  


