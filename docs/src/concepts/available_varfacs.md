
## Variables in Caesar.jl
You can check for the latest variable types by running the following in your terminal:

```julia
using RoME, Caesar
subtypes(IncrementalInference.InferenceVariable)
IncrementalInference.getCurrentWorkspaceVariables()
```

Default variables in IncrementalInference

```@docs
ContinuousScalar
ContinuousMultivariate
```

### 2D Variables

The current variables types are:
```@docs
Point2
Pose2
DynPoint2
DynPose2
```

### 3D Variables

```@docs
Point3
Pose3
InertialPose3
```

> **Note** Please open an issue with [JuliaRobotics/RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) for specific requests, problems, or suggestions.  Contributions are also welcome.

> **Note** There might be more variable types in Caesar/RoME/IIF not yet documented here.

## Factors in Caesar.jl
You can check for the latest factor types by running the following in your terminal:

```julia
using RoME, Caesar
println("- Singletons (priors): ")
println.(sort(string.(subtypes(IncrementalInference.FunctorSingleton))));
println("- Pairwise (variable constraints): ")
println.(sort(string.(subtypes(IncrementalInference.FunctorPairwise))));
println("- Pairwise (variable minimization constraints): ")
println.(sort(string.(subtypes(IncrementalInference.FunctorPairwiseMinimize))));
```

### Priors (Absolute Data)
Existing prior (unary) factors in Caesar.jl/RoME.jl/IIF.jl include:

```@docs
PriorPoint2
PriorPose2
PriorPolar
```

### Prior 3D (unary) factors

```@docs
PriorPoint3
PriorPose3
```

Defaults in IncrementalInference.jl:
```@docs
Prior
PartialPrior
MixturePrior
```

### Conditional Likelihoods (Relative Data)
Existing n-ary factors in Caesar.jl/RoME.jl/IIF.jl include:
```@docs
Point2Point2
Point2Point2WorldBearing
Pose2Point2Bearing
Pose2Point2BearingRange
Pose2Point2Range
Pose2Pose2
DynPoint2VelocityPrior
DynPoint2DynPoint2
VelPoint2VelPoint2
Point2Point2Velocity
DynPose2VelocityPrior
VelPose2VelPose2
DynPose2Pose2
Pose3Pose3
InertialPose3
PriorPose3ZRP
PartialPriorRollPitchZ
PartialPose3XYYaw
Pose3Pose3XYYaw
```

Defaults in IncrementalInference.jl:
```@docs
LinearConditional
MixtureLinearConditional
```

## Extending Caesar with New Variables and Factors
A question that frequently arises is how to design custom variables and factors to solve a specific type of graph. One strength of Caesar is the ability to incorporate new variables and factors at will. Please refer to [Adding Factors](adding_variables_factors.md) for more information on creating your own factors.
