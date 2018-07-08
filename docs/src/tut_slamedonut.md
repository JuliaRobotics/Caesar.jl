# Tutorial: Singular Ranges-only SLAM Solution (i.e. "Under-Constrained")

This tutorial describes a range-only system where there are always more variable dimensions than range measurements made.
The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point -- other examples show inference results where highly non-Gaussian error distributions are used.
The one pre-baked result of this of this singular range-only illustration can be seen in this video:

Multi-modal range only example ([click here or image for full Vimeo](http://vimeo.com/190052649)):   
```@raw html
<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/dehann/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="640" border="0" /></a>
```

## Loading The Data

Starting a Juno IDE or Julia REPL session, the ground truth positions for vehicle positions `GTp` and landmark positions `GTl` can be loaded into memory directly with these values:
```julia
GTp = Dict{Symbol, Vector{Float64}}()
GTp[:l100] = [0.0;0]
GTp[:l101] = [50.0;0]
GTp[:l102] = [100.0;0]
GTp[:l103] = [100.0;50.0]
GTp[:l104] = [100.0;100.0]
GTp[:l105] = [50.0;100.0]
GTp[:l106] = [0.0;100.0]
GTp[:l107] = [0.0;50.0]
GTp[:l108] = [0.0;-50.0]
GTp[:l109] = [0.0;-100.0]
GTp[:l110] = [50.0;-100.0]
GTp[:l111] = [100.0;-100.0]
GTp[:l112] = [100.0;-50.0]

GTl = Dict{Symbol, Vector{Float64}}()
GTl[:l1] = [10.0;30]
GTl[:l2] = [30.0;-30]
GTl[:l3] = [80.0;40]
GTl[:l4] = [120.0;-50]
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
**NOTE** Julia uses just-in-time compiling ([unless pre-compiled](https://stackoverflow.com/questions/40116045/why-is-julia-taking-a-long-time-on-the-first-call-into-my-module)), therefore each time a new function call on a Julia process will be slow, but all following calls to the same functions will be as fast as the statically compiled code.

This example exclusively uses `Point2` variable node types, which have dimension `2` and represent `[x, y]` position estimates in the world frame.

Next construct the factor graph containing the first pose `:l100` (without any knowledge of where it is) and three measured beacons/landmarks `:l1,:l2,:l3` -- with prior location knowledge for `:l1` and `:l2`:
```julia
# create the factor graph object
fg = initfg()

# first pose with no initial estimate
addNode!(fg, :l100, Point2)

# add three landmarks
addNode!(fg, :l1, Point2)
addNode!(fg, :l2, Point2)
addNode!(fg, :l3, Point2)

# and put priors on :l101 and :l102
addFactor!(fg, [:l1;], PriorPoint2D(GTl[:l1], eye(2), [1.0]))
addFactor!(fg, [:l2;], PriorPoint2D(GTl[:l2], eye(2), [1.0]))
```
The `PriorPoint2D` is assumed to be a multivariate normal distribution of covariance `eye(2)`, as well as a weighting factor of `[1.0]`.

**NOTE** Upcoming API change: `PriorPoint2D` will be changed to accept distribution objects and discard the weighting parameter (likely `RoME v0.1.5` -- see [issue 72 here](https://github.com/JuliaRobotics/RoME.jl/issues/72)).

## Adding Range Measurements Between Variables

Next we connect the three range measurements from the vehicle location `:l0` to the three beacons, respectively -- and consider that the range measurements are completely relative between the vehicle and beacon position estimates:
```julia
# first range measurement
rhoZ1 = norm(GTl[:l1]-GTp[:l100])
ppr = Point2DPoint2DRange([rhoZ1], 2.0, [1.0])
addFactor!(fg, [:l100;:l101], ppr)

# second range measurement
rhoZ2 = norm(GTl[:l2]-GTp[:l100])
ppr = Point2DPoint2DRange([rhoZ2], 3.0, [1.0])
addFactor!(fg, [:l100; :l2], ppr)

# second range measurement
rhoZ3 = norm(GTl[:l3]-GTp[:l100])
ppr = Point2DPoint2DRange([rhoZ3], 3.0, [1.0])
addFactor!(fg, [:l100; :l3], ppr)
```

The ranging measurement standard deviation of `2.0` or `3.0` is taken, assuming a Gaussian measurement assumption.  
Again, any distribution could have been used.
The factor graph should look as follows:
```julia
writeGraphPdf(fg) # show the factor graph
```

![exranges01](https://user-images.githubusercontent.com/6412556/42350352-1f36072e-807e-11e8-997b-846223cc5262.png)

## Inference and Visualizations

At this point we can call the solver start interpreting the first results:
```julia
tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)
```

The factor graph figure above showed the structure between variables and factors.
In order to see the numerical values contained in the factor graph, a set of tools are provided by the `RoMEPlotting` and `KernelDensityEstimatePlotting` packages.
For more details, please see the [dedicated visualization discussion here](http://www.juliarobotics.org/Caesar.jl/latest/arena_visualizations.html).

First look at the two landmark positions `:l1, :l2` at `(10.0,30)`,`(30.0,-30)` respectively.

```julia
using KernelDensityEstimatePlotting

plotKDE(fg, [:l1;:l2], dims=[1;2], levels=4)
```

![testl1_2](https://user-images.githubusercontent.com/6412556/42423068-ca8690c2-82c1-11e8-8f9c-d5df13cca264.png)

Similarly, the belief estimate for the first vehicle position `:l100` is bi-modal, due to the intersection of two range measurements:
```julia
plotKDE(fg, :l100, dims=[1;2])
```

![testl100](https://user-images.githubusercontent.com/6412556/42423069-d188212e-82c1-11e8-8af6-7b82f3f14030.png)

An alternative plotting interface can also be used, that shows a histogram of desired elements instead:
```julia
drawLandms(fg, from=1, to=101)
```

![testlall](https://user-images.githubusercontent.com/6412556/42423113-aec928d0-82c2-11e8-9852-c231d1e880fe.png)

Notice the ring of particles which represents the belief on the third beacon/landmark `:l3`, which was not constrained by a prior factor.
Instead, the belief over the position of `:l3` is being estimated simultaneous to estimating the vehicle position `:l100`.

## Stochastic Growth and Decay of Modes (i.e. Hypotheses)

Next consider the vehicle moving a distance of `50` units---and by design the direction of travel is not known---to the next true position.
The video above gives away the vehicle position with the cyan line, showing travel in the shape of a lower case 'e'.
Finally, to speed things up, lets write a function that handles the travel (pseudo odometry factors between positions) and ranging measurement factors to beacons.

```julia

# Check for feasible measurements:  vehicle within 150 units from the beacons/landmarks
function vehicle_drives_to!(fgl::FactorGraph, possym::Symbol, GTp::Dict, GTl::Dict; measurelimit::R=150.0) where {R <: Real}
  @show beacons = keys(GTl)
  for ll in beacons
    rho = norm(GTl[ll] - GTp[possym])
    if rho < measurelimit
      ppr = Point2DPoint2DRange([norm(GTl[ll] - GTp[possym])], 3.0, [1.0])
      if !(:l4 in union(ls(fgl)...))
        println("Adding variable vertex $ll, not yet in fgl::FactorGraph.")
        addNode!(fgl, ll, Point2, N=N, ready=0)
      end
      addFactor!(fgl, [possym;ll], ppr, ready=0)
    end
  end
  nothing
end
```

After pasting (or running) this function in the Julia, a new member definition exists for `vehicle_drives_to!`.

**NOTE** The exclamation mark at the end of the function name has no syntactic significance in Julia, since the full UTF8 character set is available for functions or variables.
Instead, the exclamation serves as a Julia community convention to tell the caller that this function will modify the contents of at least some of the variables being passed into it -- in this case the factor graph `fg` will be modified.

Now the actual driving event can be added to the factor graph:

```julia
#drive to next location :l101
vehicle_drives_to!(fg, :l101, GTp, GTl)
```


**NOTE** The distance traveled could be any combination of accrued direction and speeds, however, a straight line Gaussian error model is used to keep the visual presentation of this example as simple as possible.





**WORK IN PROGRESS -- AND DEBUGGING**
