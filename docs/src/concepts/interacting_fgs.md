# Querying the FactorGraph

There are a variety of functions to query the factor graph, please refer to [Function Reference](../func_ref.md) for details.

A quick summary of the variables in the factor graph can be retrieved with:

```julia
# List variables
ls(fg)
# List factors attached to x0
ls(fg, :x0)
# TODO: Provide an overview of getVal, getVert, getBW, getBelief, etc.
```

A factor graph object can be visualized using:
```julia
drawGraph(fg, show=true)
```

By setting `show=true`, the application `evince` will be called to show the `fg.pdf` file that was created using *GraphViz*.  A `GraphPlot.jl` visualization engine is also available.

# Solving Graphs

When you have built the graph, you can call the solver to perform inference with the following:

```julia
# Perform inference
tree, smt, hist = solveTree!(fg)
```

The returned Bayes (Junction) `tree` object is described in more detail on [a dedicated documentation page](https://juliarobotics.org/Caesar.jl/latest/principles/bayestreePrinciples/), while `smt` and `hist` return values most closely relate to development and debug outputs which can be ignored during general use.  Should an error occur during, the exception information is easily accessible in the `smt` object (as well as file logs which default to `/tmp/caesar/`).

One of the major features of the multimodal-iSAM (mmisam) algorithm (implemented by [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl)) is reducing computational load by recycling and marginalizing different (usually older) parts of the factor graph.  In order to utilize the benefits of recycing, the previous Bayes (Junction) tree should also be provided as input (see fixed-lag examples for more details):
```julia
tree, smt, hist = solveTree!(fg, tree)
```

## Peeking at Results

Once you have solved the graph, you can review the full marginal with:

```julia
X0 = getKDE(fg, :x0) # Get the raw KDE
# Evaluate the marginal density function just for fun at [0.01, 0, 0].
X0([0.01, 0, 0])
```

For finding the MAP value in the density functions, you can use `getKDEMax` or `getKDEMean`. Here we are asking for the MAP values for all the variables in the factor graph:

```julia
varsyms = ls(fg)
map(v -> println("$v : $(getKDEMax(getKDE(fg, v)))"), varsyms[1]);
```

> Also see built-in function `printgraphmax(fg)` which performs a similar function.
