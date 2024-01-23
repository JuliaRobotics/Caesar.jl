# [Solving Graphs](@id solving_graphs)
## Non-parametric Batch Solve

When you have built the graph, you can call the solver to perform inference with the following:

```julia
# Perform inference
tree = solveTree!(fg) # or solveGraph!
```

The returned Bayes (Junction) `tree` object is described in more detail on [a dedicated documentation page](https://juliarobotics.org/Caesar.jl/latest/principles/bayestreePrinciples/), while `smt` and `hist` return values most closely relate to development and debug outputs which can be ignored during general use.  Should an error occur during, the exception information is easily accessible in the `smt` object (as well as file logs which default to `/tmp/caesar/`).

```@docs
solveTree!
```

## [Automatic vs Manual Init](@id variable_init)

Currently the main automatic initialization technique used by IncrementalInference.jl by delayed propagation of belief on the factor graph.  This can be globally or locally controlled via:
```julia
getSolverParams(fg).graphinit = false

# or locally at each addFactor
addFactor!(fg, [:x0;:x1], LinearRelative(Normal()); graphinit=false)
```

Use [`initVariable!`](@ref) if you'd like to force a particular numerical initialization of some or all the variables.
```@docs
initVariable!
```

All the variables can be initialized without solving with:
```@docs
initAll!
```

## Using Incremental Updates (Clique Recycling I)

One of the major features of the MM-iSAMv2 algorithm (implemented by [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl)) is reducing computational load by recycling and marginalizing different (usually older) parts of the factor graph.  In order to utilize the benefits of recycing, the previous Bayes (Junction) tree should also be provided as input (see fixed-lag examples for more details):
```julia
tree = solveTree!(fg, tree)
```

## Using Clique out-marginalization (Clique Recycling II)

When building sysmtes with limited computation resources, the out-marginalization of cliques on the Bayes tree can be used.  This approach limits the amount of variables that are inferred on each solution of the graph.  This method is also a compliment to the above Incremental Recycling -- these two methods can work in tandem.  There is a default setting for a FIFO out-marginalization strategy (with some additional tricks):
```julia
defaultFixedLagOnTree!(fg, 50, limitfixeddown=true)
```

This call will keep the latest 50 variables fluid for inference during Bayes tree inference.  The keyword `limitfixeddown=true` in this case will also prevent downward message passing on the Bayes tree from propagating into the out-marginalized branches on the tree.  A later page in this documentation will discuss how the inference algorithm and Bayes tree aspects are put together.

## [Synchronizing Over a Factor Graph](@id sync_over_graph_solvable)

When adding Variables and Factors, use `solvable=0` to disable the new fragments until ready for inference, for example
```julia
addVariable!(fg, :x45, Pose2, solvable=0)
newfct = addFactor!(fg, [:x11,:x12], Pose2Pose2, solvable=0)
```

These parts of the factor graph can simply be activated for solving:
```julia
setSolvable!(fg, :x45, 1)
setSolvable!(fg, newfct.label, 1)
```