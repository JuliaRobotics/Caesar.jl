# Building and Solving Graphs
Irrespective of your application - real-time robotics, batch processing of survey data, or really complex multi-hypothesis modeling - you're going to need to add factors and variables to a graph. This section discusses how to do that in Caesar.

The following sections discuss the steps required to construct a graph and solve it:
* Initialing the Factor Graph
* Adding Variables and Factors to the Graph
* Solving the Graph
* Informing the Solver About Ready Data

# Initializing a Factor Graph

```julia
using Caesar, RoME, Distributions

# start with an empty factor graph object
fg = initfg()
```

## Adding to the Graph
Factor graphs are made of two constituent parts:
* Variables
* Factors

## Variables
Variables (a.k.a. poses in localization terminology) are created in the same way  shown above for the landmark. Variables contain a label, a data type (e.g. in 2D `RoME.Point2` or `RoME.Pose2`). Note that variables are solved - i.e. they are the product, what you wish to calculate when the solver runs - so you don't provide any measurements when creating them.

```julia
# Add the first pose :x0
addNode!(fg, :x0, Pose2)
# Add a few more poses
for i in 1:10
  addNode!(fg, Symbol("x$(i)"), Pose2)
end
```

## Factors
Factors are algebraic relationships between variables based on data cues such as sensor measurements. Examples of factors are absolute GPS readings (unary factors/priors) and odometry changes between pose variables. All factors encode a stochastic measurement (measurement + error), such as below, where a prior is defined against x0 with a normal distribution centered around [0,0,0].

### Priors
```julia
# Add at a fixed location Prior to pin :x0 to a starting location (0,0,pi/6.0)
addFactor!(fg, [:x0], IIF.Prior( MvNormal([0; 0; pi/6.0], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))
```

### Factors Between Variables

```julia
# Add odometry indicating a zigzag movement
for i in 1:10
  pp = Pose2Pose2(MvNormal([10.0;0; (i % 2 == 0 ? -pi/3 : pi/3)], Matrix(Diagonal([0.1;0.1;0.1].^2))))
  addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
end
```

### When to Create New Pose Variables

Consider a robot traversing some area while exploring, localizing, and wanting to find strong loop-closure features for consistent mapping.  The creation of new poses and landmark variables is a trade-off in computational complexity and marginalization errors made during factor graph construction.  Common triggers for new poses are:
- Time-based trigger (eg. new pose a second or 5 minutes if stationary)
- Distance traveled (eg. new pose every 0.5 meters)
- Rotation angle (eg. new pose every 15 degrees)

Computation will progress faster if poses and landmarks are very sparse.  To extract the benefit of dense reconstructions, one approach is to use the factor graph as sparse index in history about the general progression of the trajectory and use additional processing from dense sensor data for high-fidelity map reconstructions.  Either interpolations, or better direct reconstructions from inertial data can be used for dense reconstruction.

For completeness, one could also re-project the most meaningful measurements from sensor measurements between pose epochs as though measured from the pose epoch.  This approach essentially marginalizes the local dead reckoning drift errors into the local interpose re-projections, but helps keep the pose count low.

In addition, see [fixed-lag discussion](../examples/interm_fixedlag_hexagonal.md) for limiting during inference the number of fluid variables manually to a user desired count.

# Variables and Factors Available in Caesar

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

> **Note** several more variable and factors types have been implemented which will over time be incorporated into standard `RoME` release.  Please open an issue with [JuliaRobotics/RoME.jl](JuliaRobotics/RoME.jl) for specific requests, problems, or suggestions.  Contributions are also welcome.

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

Note: This has been made available as `IncrementalInference.getCurrentWorkspaceFactors()` in IncrementalInference v0.4.4.

The current factor types that you will find in the examples are (there are many aside from these):

* `RoME.Prior` - A singleton indicating a prior on a variable
* `RoME.Point2Point2` - A factor between two 2D points
* `RoME.Point2Point2WorldBearing` - A factor between two 2D points with bearing
* `RoME.Pose2Point2Bearing` - A factor between two 2D points with bearing
* `RoME.Pose2Point2BearingRange` - A factor between two 2D points with bearing and range
* `RoME.Pose2Point2Range` - A factor between a 2D pose and a 2D point, with range
* `RoME.Pose2Pose2` - A factor between two 2D poses
* `RoME.Pose3Pose3` - A factor between two 3D poses
* `RoME.IntertialPose3` - A factor between two 3D IMU sensor poses

# Querying the FactorGraph

There are a variety of functions to query the factor graph, please refer to [Function Reference](../func_ref.md) for details.

A quick summary of the variables in the factor graph can be retrieved with:

```julia
# List variables
ls(fg)
# List factors attached to x0
ls(fg, :x0)
# TODO: Provide an overview of getVal, getVert, getBW, getVertKDE, etc.
```

## Solving Graphs
When you have built the graph, you can call the solver to perform inference with the following:

```julia
# Perform inference
batchSolve!(fg)
```

## Peeking at Results
Once you have solved the graph, you can review the full marginal with:

```julia
X0 = getVertKDE(fg, :x0) # Get the raw KDE
# Evaluate the marginal density function just for fun at [0.01, 0, 0].
X0([0.01, 0, 0])
```

For finding the MAP value in the density functions, you can use `getKDEMax` or `getKDEMean`. Here we are asking for the MAP values for all the variables in the factor graph:

```julia
verts = ls(fg)
map(v -> println("$v : $(getKDEMax(getVertKDE(fg, v)))"), verts[1]);
```

> Also see built-in function `printgraphmax(fg)` which performs a similar function.

## Plotting
Once the graph has been built, a simple plot of the values can be produced with RoMEPlotting.jl. For example:

```julia
using RoMEPlotting

drawPoses(fg)
# If you have landmarks, you can call drawPosesLandms(fg)

# Draw the KDE for x0
plotKDE(fg, :x0)
# Draw the KDE's for x0 and x1
plotKDE(fg, [:x0, :x1])
```

## Next Steps
Although the above graph demonstrates the fundamental operations, it's not particularly useful. Take a look at [Hexagonal Example](../examples/basic_hexagonal2d.md) for a complete example that builds on these operations.

### Extending Caesar with New Variables and Factors
A question that frequently arises is how to design custom variables and factors to solve a specific type of graph. One strength of Caesar is the ability to incorporate new variables and factors at will. Please refer to [Adding Factors](adding_variables_factors.md) for more information on creating your own factors.
