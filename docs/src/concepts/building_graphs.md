# Building and Solving Graphs
Irrespective of your application - real-time robotics, batch processing of survey data, or really complex multi-hypothesis modeling - you're going to need to add factors and variables to a graph. This section discusses how to do that in Caesar.

The following sections discuss the steps required to construct a graph and solve it:
* Initialing the Factor Graph
* Adding Variables and Factors to the Graph
* Solving the Graph
* Informing the Solver About Ready Data

## Initializing a Factor Graph

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
addVariable!(fg, :x0, Pose2)
# Add a few more poses
for i in 1:10
  addVariable!(fg, Symbol("x$(i)"), Pose2)
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

### When to Instantiate Poses (i.e. new Variables in Factor Graph)

Consider a robot traversing some area while exploring, localizing, and wanting to find strong loop-closure features for consistent mapping.  The creation of new poses and landmark variables is a trade-off in computational complexity and marginalization errors made during factor graph construction.  Common triggers for new poses are:
- Time-based trigger (eg. new pose a second or 5 minutes if stationary)
- Distance traveled (eg. new pose every 0.5 meters)
- Rotation angle (eg. new pose every 15 degrees)

Computation will progress faster if poses and landmarks are very sparse.  To extract the benefit of dense reconstructions, one approach is to use the factor graph as sparse index in history about the general progression of the trajectory and use additional processing from dense sensor data for high-fidelity map reconstructions.  Either interpolations, or better direct reconstructions from inertial data can be used for dense reconstruction.

For completeness, one could also re-project the most meaningful measurements from sensor measurements between pose epochs as though measured from the pose epoch.  This approach essentially marginalizes the local dead reckoning drift errors into the local interpose re-projections, but helps keep the pose count low.

In addition, see [fixed-lag discussion](../examples/interm_fixedlag_hexagonal.md) for limiting during inference the number of fluid variables manually to a user desired count.

## Which Variables and Factors to use

See the next page on [available variables and factors](concepts/available_varfacs.md)
