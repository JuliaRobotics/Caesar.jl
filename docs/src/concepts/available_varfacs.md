# [Variables in Caesar.jl](@id variables_factors)

You can check for the latest variable types by running the following in your terminal:

```julia
using RoME, Caesar

subtypes(IIF.InferenceVariable)

# variables already available
IIF.getCurrentWorkspaceVariables()

# factors already available
IIF.getCurrentWorkspaceFactors()
```

The variables and factors in Caesar should be sufficient for a variety of robotic applications, however, users can easily extend the framework (without changing the core code).  This can even be done *out-of-library*  at runtime after a construction of a factor graph has started!  See [Custom Variables](@ref custom_variables) and [Custom Factors](@ref custom_relative_factor) for more details.

## Basic Variables

Default variables in IncrementalInference

```@docs
Position{N}
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
```

!!! note
    Please open an issue with [JuliaRobotics/RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) for specific requests, problems, or suggestions.  Contributions are also welcome.  There might be more variable types in Caesar/RoME/IIF not yet documented here.

# Factors in Caesar.jl

You can check for the latest factor types by running the following in your terminal:

```julia
using RoME, Caesar
println("- Singletons (priors): ")
println.(sort(string.(subtypes(IIF.AbstractPrior))));
println("- Pairwise (variable constraints): ")
println.(sort(string.(subtypes(IIF.AbstractRelativeRoots))));
println("- Pairwise (variable minimization constraints): ")
println.(sort(string.(subtypes(IIF.AbstractRelativeMinimize))));
```

### Priors (Absolute Data)

Defaults in IncrementalInference.jl:
```@docs
Prior
PartialPrior
```

Some of the most common priors (unary factors) in Caesar.jl/RoME.jl include:
```@docs
PriorPolar
PriorPoint2
PriorPose2
PriorPoint3
PriorPose3
```

### Relative Likelihoods (Relative Data)

Defaults in IncrementalInference.jl:
```@docs
LinearRelative
```

Existing n-ary factors in Caesar.jl/RoME.jl/IIF.jl include:
```@docs
PolarPolar
Point2Point2
Pose2Point2
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
PriorPose3ZRP
Pose3Pose3XYYaw
```
<!-- PartialPose3XYYaw -->
<!-- PartialPriorRollPitchZ -->

# Extending Caesar with New Variables and Factors

A question that frequently arises is how to design custom variables and factors to solve a specific type of graph. One strength of Caesar is the ability to incorporate new variables and factors at will. Please refer to [Adding Factors](../examples/custom_variables.md) for more information on creating your own factors.
