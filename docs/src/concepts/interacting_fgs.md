## Querying the FactorGraph

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
X0 = getKDE(fg, :x0) # Get the raw KDE
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
# If you have landmarks, you can instead call
# drawPosesLandms(fg)

# Draw the KDE for x0
plotKDE(fg, :x0)
# Draw the KDE's for x0 and x1
plotKDE(fg, [:x0, :x1])
```

## Next Steps
Although the above graph demonstrates the fundamental operations, it's not particularly useful. Take a look at [Hexagonal Example](../examples/basic_hexagonal2d.md) for a complete example that builds on these operations.
