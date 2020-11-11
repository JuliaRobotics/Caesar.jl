# Plotting

Once the graph has been built, 2D plot visualizations are provided by [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) and [KernelDensityEstimatePlotting.jl](http://www.github.com/JuliaRobotics/KernelDensityEstimatePlotting.jl).  These visualizations tools are readily modifiable to highlight various aspects of mobile platform navigation.

!!! note

    Plotting packages [can be installed separately](https://juliarobotics.org/Caesar.jl/latest/installation_environment/#RoMEPlotting.jl-for-2D-plots-1).

## Quick Start

The major 2D plotting functions between `RoMEPlotting.jl` and `KernelDensityEstimatePlotting.jl`:
- [`plotSLAM2D`](@ref),
- [`plotSLAM2DPoses`](@ref),
- [`plotSLAM2DLandmarks`](@ref),
- [`plotKDE` / `plot`](@ref).

A simple usage example:

```julia
using RoMEPlotting

plotPoses(fg)
# If you have landmarks, you can instead call
# plotSLAM2D(fg)

# Draw the KDE for x0
plotKDE(fg, :x0)
# Draw the KDE's for x0 and x1
plotKDE(fg, [:x0, :x1])
```

## Hexagonal 2D SLAM example visualization

This simplest example for visualizing a 2D robot trajectory---such as first running [the Hexagonal 2D SLAM example](http://www.juliarobotics.org/Caesar.jl/latest/tut_hexagonal2d.html)---
```julia
# Assuming some fg<:AbstractDFG has been loaded/constructed
# ...

using RoMEPlotting
using Gadfly
# VSCode/Juno can set plot to be opened in a browser tab instead, and this will change the default plot size
# Gadfly.set_default_plot_size(35cm, 30cm)

# generate a slam2d plot
pl = plotSLAM2D(fg)

# For scripting use-cases you can also export the image
pl |> PDF("/tmp/test.pdf", 20cm, 10cm)  # or PNG, SVG
```

![test](https://user-images.githubusercontent.com/6412556/69353457-6cd82400-0c76-11ea-905c-8f435faa6b11.png)

## Density Contour Map

`KernelDensityEstimatePlotting` (as used in `RoMEPlotting`) provides an interface to visualize belief densities as counter plots.
The following basic example shows some of features of the API, where `plotKDE(..., dims=[1;2])` implies the marginal over variables `(x,y)`:

```julia
using RoME, Distributions
using RoMEPlotting

fg = initfg()
addVariable!(fg, :x0, Pose2)
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), diagm([1;1;1.0]))))
addVariable!(fg, :x1, Pose2)
addFactor!(fg, [:x0;:x1], Pose2Pose2(MvNormal([10.0;0;0], diagm([1;1;1.0]))))

ensureAllInitialized!(fg)

# plot one contour density
plX0 = plotKDE(fg, :x0, dims=[1;2])
# using Gadfly; Gadfly.draw(PNG("/tmp/testX0.png",20cm,10cm),plX0)
```

![test](https://user-images.githubusercontent.com/6412556/42532654-93f9a87e-8455-11e8-9dc7-b00f73f1321a.png)

The contour density relates to the distribution of marginal samples as seen with this [Gadfly.jl package](http://gadflyjl.org/stable/) histogram comparison.

```julia
pl1 = plotSLAM2DPoses(fg, to=0);
X0 = getBelief(fg, :x0) |> getPoints;
pl2 = Gadfly.plot(x=X0[1,:],y=X0[2,:], Geom.hexbin);
plH = hstack(pl1, pl2)

# convert to file
# p1H |> PNG("/tmp/testH.png")
```

![testh](https://user-images.githubusercontent.com/6412556/42533539-2c8571e8-8458-11e8-86f6-39d1e5c94242.png)

!!! note
    Red and Green lines represent Port and Starboard direction of `Pose2`, respectively.

Multiple beliefs can be plotted at the same time, while setting `levels=4` rather than the default value:

```julia
plX1 = plotKDE(fg, [:x0; :x1], dims=[1;2], levels=4)

# plX1 |> PNG("/tmp/testX1.png")
```

![testx1](https://user-images.githubusercontent.com/6412556/42532963-656cef56-8456-11e8-9636-42592c0d148c.png)

One dimensional (such as `Θ`) or a stack of all plane projections is also available:

```julia
plTh = plotKDE(fg, [:x0; :x1], dims=[3], levels=4)

# plTh |> PNG("/tmp/testTh.png")
```

![testth](https://user-images.githubusercontent.com/6412556/42533188-2dee90c4-8457-11e8-9844-0ef57fba1c82.png)

```julia
plAll = plotKDE(fg, [:x0; :x1], levels=3)
# plAll |> PNG("/tmp/testX1.png",20cm,15cm)
```

![testall](https://user-images.githubusercontent.com/6412556/42533225-42ddaf9c-8457-11e8-8b0d-b1f3695d8b00.png)

!!! note

    The functions `hstack` and `vstack` is provided through the `Gadfly` package and allows the user to build a near arbitrary composition of plots.

Please see [KernelDensityEstimatePlotting package source](https://github.com/JuliaRobotics/KernelDensityEstimatePlotting.jl) for more features.

## Interactive Gadfly.jl Plots

See the following two discussions on Interactive 2D plots:
- [Interactivity](http://gadflyjl.org/stable/tutorial/#Interactivity-1)
- [Interactive-SVGs](http://gadflyjl.org/stable/man/backends/#Interactive-SVGs-1)
