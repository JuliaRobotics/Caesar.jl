
## Variables and Factors Already Available in Caesar

### Variables Available in Caesar
You can check for the latest variable types by running the following in your terminal:

```julia
using RoME, Caesar
subtypes(IncrementalInference.InferenceVariable)
```

Note: This has been made available as `IncrementalInference.getCurrentWorkspaceVariables()` in IncrementalInference v0.4.4.

The current list of available variable types is:
* `RoME.Point2` - A 2D coordinate consisting of [x, y, theta]
* `RoME.Pose2` - A 2D coordinate and a rotation (i.e. bearing) consisting of [x, y, z, and theta]
* `RoME.DynPoint2` - A 2D coordinate and linear velocities
* `RoME.DynPose2` - A 2D coordinate, linear velocities, and a rotation
* `RoME.Point3` - A 3D coordinate consisting of [x, y, z]
* `RoME.Pose3` - A 3D coordinate and 3 associated rotations consisting of [x, y, z, theta, phi, psi]
* `RoME.InertialPose3` - A 3D coordinate and rotation pose along with velocity and IMU bias calibration terms

> **Note** several more variable and factors types have been implemented which will over time be incorporated into standard `RoME` release.  Please open an issue with [JuliaRobotics/RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) for specific requests, problems, or suggestions.  Contributions are also welcome.

### Factors Available in Caesar
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

Existing prior (unary) factors in Caesar.jl/RoME.jl/IIF.jl include:

```@docs
Prior
PriorPoint2
PriorPose2
PriorPolar
PriorPoint3
PriorPose3
```

Existing n-ary factors in Caesar.jl/RoME.jl/IIF.jl include:

```@docs
Point2Point2
Point2Point2WorldBearing
Pose2Point2Bearing
Pose2Point2BearingRange
Pose2Point2Range
Pose2Pose2
Pose3Pose3
InertialPose3
```

### Extending Caesar with New Variables and Factors
A question that frequently arises is how to design custom variables and factors to solve a specific type of graph. One strength of Caesar is the ability to incorporate new variables and factors at will. Please refer to [Adding Factors](adding_variables_factors.md) for more information on creating your own factors.
