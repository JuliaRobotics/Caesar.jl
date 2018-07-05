# Tutorial: Singular Ranges-only SLAM Solution (i.e. "Under-Constrained")

This tutorial describes a range-only system where there are always more variable dimensions than range measurements made.
The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point -- other examples show inference results where highly non-Gaussian error distributions are used.
The one pre-baked result of this of this singular range-only illustration can be seen in this video:

Multi-modal range only example:   
```@raw html
<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/dehann/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="640" border="0" /></a>
```

## Loading The Data

Starting a Juno IDE or Julia REPL session, the ground truth positions for vehicle positions `GTp` and landmark positions `GTl` can be loaded into memory directly with these values:
```julia
GTp = Dict{Symbol, Vector{Float64}}()
GTp[:l0] = [0.0;0]
GTp[:l1] = [50.0;0]
GTp[:l2] = [100.0;0]
GTp[:l3] = [100.0;50.0]
GTp[:l4] = [100.0;100.0]
GTp[:l5] = [50.0;100.0]
GTp[:l6] = [0.0;100.0]
GTp[:l7] = [0.0;50.0]
GTp[:l8] = [0.0;-50.0]
GTp[:l9] = [0.0;-100.0]
GTp[:l10] = [50.0;-100.0]
GTp[:l11] = [100.0;-100.0]
GTp[:l12] = [100.0;-50.0]

GTl = Dict{Symbol, Vector{Float64}}()
GTl[:l101] = [10.0;30]
GTl[:l102] = [30.0;-30]
GTl[:l103] = [80.0;40]
GTl[:l104] = [120.0;-50]
```

**NOTE 1.** that by using location indicators `:l1, :l2, ...` or `:l100, :l101, ...` is of practical benefit when visualizing with existing [RoMEPlotting](https://github.com/JuliaRobotics/RoMEPlotting.jl) functions.

**NOTE 2.** Landmarks must be in range before range measurements can be made to them.

## Creating the Factor Graph with `Point2`

The first step is to load the required modules, and in our case we will add a few Julia processes to help with the compute later on.  
```julia
# add more julia processes
nprocs() < 7 ? addprocs(7-nprocs()) : nothing

# tell Julia that you want to use these modules/namespaces
using RoME, Distributions
```
**NOTE** Julia uses just-in-time compiling each time a new function is called on each Julia process -- i.e. first time calls will be slow, but all following calls to the same function will be as fast as the static compiled code allows.

This example exclusively uses `Point2` variable node types, which have dimension `2` and represent `[x, y]` position estimates in the world frame.

Next construct the factor graph containing the first `:l0` pose (without any knowledge of where it is) and three measured landmarks `:l101,:l102,:l103` -- with knowledge of where `:l101` and `:l102` is:
```julia
# create the factor graph object
fg = initfg()

# first pose with no initial estimate
addNode!(fg, :l0, Point2)

# add three landmarks
addNode!(fg, :l101, Point2)
addNode!(fg, :l102, Point2)
addNode!(fg, :l103, Point2)

# and put priors on :l101 and :l102
addFactor!(fg, [:l101;], PriorPoint2D(GTl[:l101], eye(2), [1.0]))
addFactor!(fg, [:l102;], PriorPoint2D(GTl[:l102], eye(2), [1.0]))
```
The `PriorPoint2D` is assumed to be a multivariate normal distribution of covariance `eye(2)`, as well as a weighting factor of `[1.0]`.

**NOTE** Upcoming API change: `PriorPoint2D` will be changed to accept distribution objects and discard the weigthing parameter (likely `RoME v0.1.5` -- see [issue 72 here](https://github.com/JuliaRobotics/RoME.jl/issues/72)).

Next we connect the three range measurements from the vehicle location `:l0` to the three beacons, respectively -- and consider that the range measurements are completely relative between the vehicle and beacon position estimates:
```julia
# first range measurement
rhoZ1 = norm(GTl[:l101]-GTp[:l0])
ppr = Point2DPoint2DRange([rhoZ1], 2.0, [1.0])
addFactor!(fg, [:l0;:l101], ppr)

# second range measurement
rhoZ2 = norm(GTl[:l102]-GTp[:l0])
ppr = Point2DPoint2DRange([rhoZ2], 3.0, [1.0])
addFactor!(fg, [:l0; :l102], ppr)

# second range measurement
rhoZ3 = norm(GTl[:l103]-GTp[:l0])
ppr = Point2DPoint2DRange([rhoZ3], 3.0, [1.0])
addFactor!(fg, [:l0; :l103], ppr)
```

The ranging measurement standard deviation of `2.0` or `3.0` is taken, assuming a Gaussian measurement assumption.  
Again, any distribution could have been used.
The factor graph should look as follows:
```julia
writeGraphPdf(fg) # show the factor graph
```

![exranges01](https://user-images.githubusercontent.com/6412556/42350352-1f36072e-807e-11e8-997b-846223cc5262.png)

At this point we can call the solver start interpreting the first results:
```julia
tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)
```

**WORK IN PROGRESS -- AND DEBUGGING**
