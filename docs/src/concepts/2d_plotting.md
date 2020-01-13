# Plotting

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
