# Plotting

Once the graph has been built, 2D plot visualizations are provided by [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) and [KernelDensityEstimatePlotting.jl](http://www.github.com/JuliaRobotics/KernelDensityEstimatePlotting.jl).  These visualizations tools are readily modifiable to highlight various aspects of mobile platform navigation.

!!! note
    Plotting packages [can be installed separately](https://juliarobotics.org/Caesar.jl/latest/installation_environment/#RoMEPlotting.jl-for-2D-plots-1).

The major 2D plotting functions between `RoMEPlotting.jl` and `KernelDensityEstimatePlotting.jl`:
- [`plotSLAM2D`](@ref),
- [`plotSLAM2DPoses`](@ref),
- [`plotSLAM2DLandmarks`](@ref),
- [`plotPose`](@ref),
- [`plotKDE`](@ref) / `plot`,
- [`plotLocalProduct`](@ref),
- `PDF`, `PNG`, `SVG`,
- `hstack`, `vstack`.

## Example Plot SLAM 2D

This simplest example for visualizing a 2D robot trajectory---such as first running [the Hexagonal 2D SLAM example](http://www.juliarobotics.org/Caesar.jl/latest/tut_hexagonal2d.html)---

Assuming some `fg<:AbstractDFG` has been loaded/constructed:
```julia
# load the plotting functionality
using RoME, RoMEPlotting

# generate some factor graph with numerical values
fg = generateCanonicalFG_Hexagonal()
solveTree!(fg)

# or fg = loadDFG("somepath")

# slam2D plot
pl = plotSLAM2D(fg, drawhist=true, drawPoints=false)
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/69353457-6cd82400-0c76-11ea-905c-8f435faa6b11.png" width="800" border="0" />
</p>
```

```@docs
plotSLAM2D
```

### Plot Covariance Ellipse and Points

While the Caesar.jl framework is focussed on non-Gaussian inference, it is frequently desirable to relate the results to a more familiar covariance ellipse, and native support for this exists:
```julia
plotSLAM2D(fg, contour=false, drawEllipse=true, drawPoints=true)
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/98863019-c4e2da00-2435-11eb-8e50-4a34cc8de2d7.png" width="800" border="0" />
</p>
```

### Plot Poses or Landmarks

Lower down utility functions are used to plot poses and landmarks separately before joining the Gadfly layers.

```@docs
plotSLAM2DPoses
plotSLAM2DLandmarks
```

## Plot Belief Density Contour

`KernelDensityEstimatePlotting` (as used in `RoMEPlotting`) provides an interface to visualize belief densities as counter plots.  Something basic might be to just show all plane pairs of this variable marginal belief:
```julia
# Draw the KDE for x0
plotKDE(fg, :x0)
```

Plotting the marginal density over say variables `(x,y)` in a [`Pose2`](@ref) would be:
```julia
plotKDE(fg, :x1, dims=[1;2])
```

The following example better shows some of features (via [Gadfly.jl](http://gadflyjl.org/stable/)):
```julia
# Draw the (x,y) marginal estimated belief contour for :x0, :x2, and Lx4
pl = plotKDE(fg, [:x0; :x2; :x4], c=["red";"green";"blue"], levels=2, dims=[1;2])

# add a few fun layers
pl3 = plotSLAM2DPoses(fg, regexPoses=r"x\d", from=3, to=3, contour=false, drawEllipse=true)
pl5 = plotSLAM2DPoses(fg, regexPoses=r"x\d", from=5, to=5, contour=false, drawEllipse=true, drawPoints=false)
pl_ = plotSLAM2DPoses(fg, contour=false, drawPoints=false, dyadScale=0.001, to=5)
union!(pl.layers, pl3.layers)
union!(pl.layers, pl5.layers)
union!(pl.layers, pl_.layers)

# change the plotting coordinates
pl.coord = Coord.Cartesian(xmin=-10,xmax=20, ymin=-1, ymax=25)

# save the plot to SVG and giving dedicated (although optional) sizing
pl |> SVG("/tmp/test.svg", 25cm, 15cm)

# also display the plot live
pl
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/98865698-c910f680-2439-11eb-8adf-e50ec37eacc2.png" width="800" border="0" />
</p>
```

See function documentation for more details on API features
```@docs
plotKDE
```

#### Save Plot to Image

VSCode/Juno can set plot to be opened in a browser tab instead.  For scripting use-cases you can also export the image:
```julia
using Gadfly
# can change the default plot size
# Gadfly.set_default_plot_size(35cm, 30cm)

pl |> PDF("/tmp/test.pdf", 20cm, 10cm)  # or PNG, SVG
```

#### Save Plot Object To File

It is also possible to store the whole plot container to file using [`JLD2.jl`](https://github.com/JuliaIO/JLD2.jl):
```julia
JLD2.@save "/tmp/myplot.jld2" pl

# and loading elsewhere
JLD2.@load "/tmp/myplot.jld2" pl
```

#### Interactive Plots, Zoom, Pan (Gadfly.jl)

See the following two discussions on Interactive 2D plots:
- [Interactivity](http://gadflyjl.org/stable/tutorial/#Interactivity-1)
- [Interactive-SVGs](http://gadflyjl.org/stable/man/backends/#Interactive-SVGs-1)

!!! note
    Red and Green dyad lines represent the visualization-only assumption of X-forward and Y-left direction of `Pose2`.  The inference and manifold libraries surrounding Caesar.jl are agnostic to any particular choice of reference frame alignment, such as north east down (NED) or forward left up (common in mobile robotics).

!!! note
    Also see [Gadfly.jl](http://gadflyjl.org/stable/) notes about `hstack` and `vstack` to combine plots side by side or vertically.

### Plot Pose Individually

It is also possible to plot the belief density of a [`Pose2`](@ref) on-manifold:
```julia
plotPose(fg, :x6)
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/98864183-7f271100-2437-11eb-8422-8ed49c1186b9.png" width="800" border="0" />
</p>
```

```@docs
plotPose
```

### Debug With Local Graph Product Plot

One useful function is to check that data in the factor graph makes sense.  While the full inference algorithm uses a Bayes (Junction) tree to assemble marginal belief estimates in an efficient manner, it is often useful for a straight forward graph based sanity check.  The [`plotLocalProduct`](@ref) projects through [`approxConv`](@ref) each of the factors connected to the target variable and plots the result.  This example looks at the loop-closure point around `:x0`, which is also pinned down by the only prior in the canonical Hexagonal factor graph.
```julia
@show ls(fg, :x0);
# ls(fg, :x0) = [:x0f1, :x0x1f1, :x0l1f1]

pl = plotLocalProduct(fg, :x0, dims=[1;2], levels=1)
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/98868152-b00a4480-243d-11eb-83fe-8630d64355ee.png" width="800" border="0" />
</p>
```

While perhaps a little cluttered to read at first, this figure shows that a new calculation local to only the factor graph `prod` in greem matches well with the existing value `curr` in red in the `fg` from the earlier `solveTree!` call.  These values are close to the prior prediction `:x0f1` in blue (fairly trivial case), while the odometry `:x0x1f1` and landmark sighting projection `:x0l1f1` are also well in agreement.

```@docs
plotLocalProduct
```

### More Detail About Density Plotting

Multiple beliefs can be plotted at the same time, while setting `levels=4` rather than the default value:

```julia
plX1 = plotKDE(fg, [:x0; :x1], dims=[1;2], levels=4)

# plX1 |> PNG("/tmp/testX1.png")
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/42532963-656cef56-8456-11e8-9636-42592c0d148c.png" width="800" border="0" />
</p>
```

One dimensional (such as `Θ`) or a stack of all plane projections is also available:

```julia
plTh = plotKDE(fg, [:x0; :x1], dims=[3], levels=4)

# plTh |> PNG("/tmp/testTh.png")
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/42533188-2dee90c4-8457-11e8-9844-0ef57fba1c82.png" width="800" border="0" />
</p>
```

```julia
plAll = plotKDE(fg, [:x0; :x1], levels=3)
# plAll |> PNG("/tmp/testX1.png",20cm,15cm)
```

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/42533225-42ddaf9c-8457-11e8-8b0d-b1f3695d8b00.png" width="800" border="0" />
</p>
```

!!! note

    The functions `hstack` and `vstack` is provided through the `Gadfly` package and allows the user to build a near arbitrary composition of plots.

Please see [KernelDensityEstimatePlotting package source](https://github.com/JuliaRobotics/KernelDensityEstimatePlotting.jl) for more features.

