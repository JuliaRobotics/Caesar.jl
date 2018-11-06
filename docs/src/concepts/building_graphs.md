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
Variables (a.k.a. poses in localization terminology) are created in the same way  shown above for the landmark. Variables contain a label, a data type (e.g. a 2D Point or Pose). Note that variables are solved - i.e. they are the product, what you wish to calculate when the solver runs - so you don't provide any measurements when creating them.

```julia
# Add the first pose :x0
addNode!(fg, :x0, Pose2)
# Add another pose x1
addNode!(fg, :x1, Pose2)
```

### Variables Available in Caesar
You can check for the latest variable types by running the following in your terminal:

```julia
using RoME, Caesar
subtypes(IncrementalInference.InferenceVariable)
```

Note: This has been made available as `IncrementalInference.getCurrentWorkspaceVariables()` in IncrementalInference v0.4.4.

The current list of available variable types is:
* Point2 - A 2D coordinate consisting of [x, y, theta]
* Point3 - A 3D coordinate consisting of [x, y, z]
* Pose2 - A 2D coordinate and a rotation (i.e. bearing) consisting of [x, y, z, and theta]
* Pose3 - A 3D coordinate and 3 associated rotations consisting of [x, y, z, theta, phi, psi]
* DynPoint2 - A 2D coordinate and linear velocities
* DynPose2 - A 2D coordinate, linear velocities, and a rotation

## Factors
Factors are algebraic relationships between variables based on data cues such as sensor measurements. Examples of factors are absolute GPS readings (unary factors/priors) and odometry changes between pose variables. All factors encode a stochastic measurement (measurement + error), such as below, where a prior is defined against x0 with a normal distribution centered around [0,0,0].

### Priors
```julia
# Add at a fixed location PriorPose2 to pin :x0 to a starting location (10,10, pi/4)
addFactor!(fg, [:x0], IIF.Prior( MvNormal([0; 0; 0], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))
```

### Factors Between Variables

```julia
pp = Pose2Pose2(MvNormal([10.0;0;pi/3], Matrix(Diagonal([0.1;0.1;0.1].^2))))
addFactor!(fg, [x0;x1], pp )

```

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

* Prior - A singleton indicating a prior on a variable
* Point2Point2 - A factor between two 2D points
* Point2Point2WorldBearing - A factor between two 2D points with bearing
* Pose2Point2Bearing - A factor between two 2D points with bearing
* Pose2Point2BearingRange - A factor between two 2D points with bearing and range
* Pose2Point2Range - A factor between a 2D pose and a 2D point, with range
* Pose2Pose2 - A factor between two 2D poses
* Pose3Pose3 - A factor between two 3D poses

## Solving Graphs

```julia
# Perform inference
batchSolve!(fg)
```

### Extending Caesar with New Variables and Factors
A question that frequently arises is how to design custom variables and factors to solve a specific type of graph. One strength of Caesar is the ability to incorporate new variables and factors at will. Please refer to [Adding Factors](adding_variables_factors.md) for more information on creating your own factors.
