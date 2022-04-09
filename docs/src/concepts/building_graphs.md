# [Building Graphs](@ref building_graphs)

Irrespective of your application - real-time robotics, batch processing of survey data, or really complex multi-hypothesis modeling - you're going to need to add factors and variables to a graph. This section discusses how to do that in Caesar.

The following sections discuss the steps required to construct a graph and solve it:
* Initializing the Factor Graph
* Adding Variables and Factors to the Graph
* Solving the Graph
* Informing the Solver About Ready Data

## Familiar Canonical Factor Graphs

Starting with a shortcut to just quickly getting a small predefined *canonical* graph containing a few variables and factors.  Functions to *generate* a canonical factor graph object that is useful for orientation, testing, learning, or validation.  You can generate any of these factor graphs at any time, for example when quickly wanting to test some idea midway through building a more sophisiticated `fg`, you might just want to quickly do:
```julia
fg_ = generateGraph_Hexagonal()
```

and then work with `fg_` to try out something risky.

!!! note
    See the [Canonical Graphs](../examples/canonical_graphs.md) page for a more complete list of existing graph generators.

## Building a new Graph

The first step is to model the data (using the most appropriate *factors*) among *variables* of interest.  To start model, first create a *distributed factor graph object*:

```julia
# start with an empty factor graph object
fg = initfg()
```

```@docs
initfg
```

## Variables

Variables (a.k.a. poses or states in navigation lingo) are created with the `addVariable!` fucntion call.

```julia
# Add the first pose :x0
addVariable!(fg, :x0, Pose2)
# Add a few more poses
for i in 1:10
  addVariable!(fg, Symbol("x",i), Pose2)
end
```

Variables contain a label, a data type (e.g. in 2D `RoME.Point2` or `RoME.Pose2`). Note that variables are solved - i.e. they are the product, what you wish to calculate when the solver runs - so you don't provide any measurements when creating them.

```@docs
addVariable!
deleteVariable!
```

### Initializing Variables

The MM-iSAMv2 algorithm uses one of two approaches to automatically initialize variables.  The `initManual!` function can be used if you wish to overwrite or pre-empt this initialization.

```@docs
initManual!
```

## Factors

Factors are algebraic relationships between variables based on data cues such as sensor measurements. Examples of factors are absolute (pre-resolved) GPS readings (unary factors/priors) and odometry changes between pose variables. All factors encode a stochastic measurement (measurement + error), such as below, where a generic [`Prior`](@ref) belief is add to `x0` (using the [`addFactor!`](@ref) call) as a normal distribution centered around `[0,0,0]`.

```@docs
addFactor!
deleteFactor!
```

### Priors
```julia
# Add at a fixed location Prior to pin :x0 to a starting location (0,0,pi/6.0)
addFactor!(fg, [:x0], PriorPose2( MvNormal([0; 0; pi/6.0], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))
```

### Factors Between Variables

```julia
# Add odometry indicating a zigzag movement
for i in 1:10
  pp = Pose2Pose2(MvNormal([10.0;0; (i % 2 == 0 ? -pi/3 : pi/3)], Matrix(Diagonal([0.1;0.1;0.1].^2))))
  addFactor!(fg, [Symbol("x$(i-1)"); Symbol("x$(i)")], pp )
end
```

#### [OPTIONAL] Understanding Internal Factor Naming Convention

The factor name used by Caesar is automatically generated from 
```julia
addFactor!(fg, [:x0; :x1],...)
```
will create a factor with name `:x0x1f1`

When you were to add a another factor betweem `:x0`, `:x1`:
```julia
addFactor!(fg, [:x0; :x1],...)
```
will create a second factor with the name `:x0x1f2`.

### Adding Tags

It is possible to add `tags` to variables and factors that make later graph management tasks easier, e.g.:
```julia
addVariable!(fg, :l7_3, Pose2, tags=[:APRILTAG; :LANDMARK])
```

### Drawing the Factor Graph

Once you have a graph, you can visualize the graph as follows (beware though if the fg object is large):
```julia
# requires `sudo apt-get install graphviz
drawGraph(fg, show=true)
```

By setting `show=true`, the application `evince` will be called to show the `fg.pdf` file that was created using *GraphViz*.  A `GraphPlot.jl` visualization engine is also available.
```julia
using GraphPlot
plotDFG(fg)
```

```@docs
drawGraph
```

For more details, see [the DFG docs on Drawing Graphs](https://juliarobotics.org/DistributedFactorGraphs.jl/latest/DrawingGraphs/#Drawing-Graphs-1).

## When to Instantiate Poses (i.e. new Variables in Factor Graph)

Consider a robot traversing some area while exploring, localizing, and wanting to find strong loop-closure features for consistent mapping.  The creation of new poses and landmark variables is a trade-off in computational complexity and marginalization errors made during factor graph construction.  Common triggers for new poses are:
- Time-based trigger (eg. new pose a second or 5 minutes if stationary)
- Distance traveled (eg. new pose every 0.5 meters)
- Rotation angle (eg. new pose every 15 degrees)

Computation will progress faster if poses and landmarks are very sparse.  To extract the benefit of dense reconstructions, one approach is to use the factor graph as sparse index in history about the general progression of the trajectory and use additional processing from dense sensor data for high-fidelity map reconstructions.  Either interpolations, or better direct reconstructions from inertial data can be used for dense reconstruction.

For completeness, one could also re-project the most meaningful measurements from sensor measurements between pose epochs as though measured from the pose epoch.  This approach essentially marginalizes the local dead reckoning drift errors into the local interpose re-projections, but helps keep the pose count low.

In addition, see [Fixed-lag Solving](@ref fixedlag_solving) for limiting during inference the number of fluid variables manually to a user desired count.

## Which Variables and Factors to use

See the next page on [available variables and factors](@ref variables_factors)
