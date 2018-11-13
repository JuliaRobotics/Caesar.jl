# Singular Ranges-only SLAM Solution (i.e. "Under-Constrained")


This tutorial describes a range-only system where there are always more variable dimensions than range measurements made.
The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point -- other examples show inference results where highly non-Gaussian error distributions are used.
The one pre-baked result of this of this singular range-only illustration can be seen in this video:

Multi-modal range only example ([click here or image for full Vimeo](http://vimeo.com/190052649)):   
```@raw html
<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="640" border="0" /></a>
```

This example is also available as a script [here in RoME.jl](https://github.com/JuliaRobotics/RoME.jl/blob/master/examples/RangesExample.jl).

## REQUIRES

- `RoME v0.2.5`
- `RoMEPlotting v0.0.2`

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
addFactor!(fg, [:l1;], PriorPoint2(MvNormal(GTl[:l1], eye(2))) )
addFactor!(fg, [:l2;], PriorPoint2(MvNormal(GTl[:l2], eye(2))) )
```
The `PriorPoint2` is assumed to be a multivariate normal distribution of covariance `eye(2)`, as well as a weighting factor of `[1.0]`.


**NOTE** API changed to `PriorPoint2(::T) where T <: SamplableBelief = PriorPoint2{T}` to accept distribution objects and discard (standard in `RoME v0.1.5` -- see [issue 72 here](https://github.com/JuliaRobotics/RoME.jl/issues/72)).

## Adding Range Measurements Between Variables

Next we connect the three range measurements from the vehicle location `:l0` to the three beacons, respectively -- and consider that the range measurements are completely relative between the vehicle and beacon position estimates:
```julia
# first range measurement
rhoZ1 = norm(GTl[:l1]-GTp[:l100])
ppr = Point2Point2Range( Normal(rhoZ1, 2.0) )
addFactor!(fg, [:l100;:l101], ppr)

# second range measurement
rhoZ2 = norm(GTl[:l2]-GTp[:l100])
ppr = Point2Point2Range( Normal(rhoZ2, 3.0) )
addFactor!(fg, [:l100; :l2], ppr)

# second range measurement
rhoZ3 = norm(GTl[:l3]-GTp[:l100])
ppr = Point2Point2Range( Normal(rhoZ3, 3.0) )
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
using RoMEPlotting

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
function vehicle_drives_to!(fgl::FactorGraph, pos_sym::Symbol, GTp::Dict, GTl::Dict; measurelimit::R=150.0) where {R <: Real}
  currvar = union(ls(fgl)...)
  prev_sym = Symbol("l$(maximum(Int[parse(Int,string(currvar[i])[2:end]) for i in 2:length(currvar)]))")
  if !(pos_sym in currvar)
    println("Adding variable vertex $pos_sym, not yet in fgl::FactorGraph.")
    addNode!(fgl, pos_sym, Point2)
    @show rho = norm(GTp[prev_sym] - GTp[pos_sym])
    ppr = Point2Point2Range( Normal(rho, 3.0) )
    addFactor!(fgl, [prev_sym;pos_sym], ppr)
  else
    @warn "Variable node $pos_sym already in the factor graph."
  end
  beacons = keys(GTl)
  for ll in beacons
    rho = norm(GTl[ll] - GTp[pos_sym])
    # Check for feasible measurements:  vehicle within 150 units from the beacons/landmarks
    if rho < measurelimit
      ppr = Point2Point2Range( Normal(rho, 3.0) )
      if !(ll in currvar)
        println("Adding variable vertex $ll, not yet in fgl::FactorGraph.")
        addNode!(fgl, ll, Point2)
      end
      addFactor!(fgl, [pos_sym;ll], ppr)
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
#drive to location :l101, then :l102
vehicle_drives_to!(fg, :l101, GTp, GTl)
vehicle_drives_to!(fg, :l102, GTp, GTl)

# see the graph
writeGraphPdf(fg)
```

**NOTE** The distance traveled could be any combination of accrued direction and speeds, however, a straight line Gaussian error model is used to keep the visual presentation of this example as simple as possible.

The marginal posterior estimates are found by repeating inference over the factor graph, followed drawing all vehicle locations as a contour map:

```julia
tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)

# draw all vehicle locations
pl = plotKDE(fg, [Symbol("l$(100+i)") for i in 0:2], dims=[1;2])
# Gadfly.draw(PDF("/tmp/testL100_102.pdf", 20cm, 10cm),pl) # for storing image to disk

pl = plotKDE(fg, [:l3;:l4], dims=[1;2], levels=4)
# Gadfly.draw(PNG("/tmp/testL3_4.png", 20cm, 10cm),pl)
```

Notice how the vehicle positions have two hypotheses, one left to right and one diagonal right to bottom left -- both are valid solutions!

![testl100_102](https://user-images.githubusercontent.com/6412556/42428772-722555b6-8303-11e8-9931-d3e2e89e3206.png)

The two "free" beacons/landmarks `:l3,:l4` still have several modes each, implying insufficient data to constrain either to a strong unimodal belief.

![testl3_4](https://user-images.githubusercontent.com/6412556/42428800-a5ac86f2-8303-11e8-984c-8952f7cdf839.png)

```julia

vehicle_drives_to!(fg, :l103, GTp, GTl)
vehicle_drives_to!(fg, :l104, GTp, GTl)

tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)

pl = plotKDE(fg, [Symbol("l$(100+i)") for i in 0:4], dims=[1;2])
# Gadfly.draw(PDF("/tmp/testL100_104.pdf", 20cm, 10cm),pl)
```

Moving up to position `:l104` still shows strong multiodality in the vehicle position estimates:

![testl100_105](https://user-images.githubusercontent.com/6412556/42428903-2f0b7e4e-8304-11e8-94b2-44fbee4d1961.png)

```julia
vehicle_drives_to!(fg, :l105, GTp, GTl)
vehicle_drives_to!(fg, :l106, GTp, GTl)

tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)


vehicle_drives_to!(fg, :l107, GTp, GTl)

tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)


vehicle_drives_to!(fg, :l108, GTp, GTl)

tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)


pl = plotKDE(fg, [Symbol("l$(100+i)") for i in 2:8], dims=[1;2], levels=6)
# Gadfly.draw(PDF("/tmp/testL103_108.pdf", 20cm, 10cm),pl)
```

Next we see a strong return to a single dominant mode in all vehicle position estimates, owing to the increased measurements to beacons/landmarks as well as more unimodal estimates in `:l3, :l4` beacon/landmark positions.

```julia
vehicle_drives_to!(fg, :l109, GTp, GTl)
vehicle_drives_to!(fg, :l110, GTp, GTl)

tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree)


vehicle_drives_to!(fg, :l111, GTp, GTl)
vehicle_drives_to!(fg, :l112, GTp, GTl)

tree = wipeBuildNewTree!(fg)
inferOverTree!(fg, tree, N=200)


pl = plotKDE(fg, [Symbol("l$(100+i)") for i in 7:12], dims=[1;2])
# Gadfly.draw(PDF("/tmp/testL106_112.pdf", 20cm, 10cm),pl)

pl = plotKDE(fg, [:l1;:l2;:l3;:l4], dims=[1;2], levels=4)
# Gadfly.draw(PDF("/tmp/testL1234.pdf", 20cm, 10cm),pl)

pl = drawLandms(fg, from=100)
# Gadfly.draw(PDF("/tmp/testLocsAll.pdf", 20cm, 10cm),pl)
```

Several location belief estimates exhibit multimodality as the trajectory progresses (not shown), but collapses and finally collapses to a stable set of dominant position estimates.

![testl106_112](https://user-images.githubusercontent.com/6412556/42429138-7576dea4-8305-11e8-9a0c-a56984805126.png)

Landmark estimates are also stable at one estimate:

![testl1234](https://user-images.githubusercontent.com/6412556/42429149-85ee3bf6-8305-11e8-8a39-6af5b7496f3c.png)

In addition, the SLAM 2D landmark visualization can be re-used to plot more information at once:

```julia
# pl = drawLandms(fg, from=100, to=200)
# Gadfly.draw(PDF("/tmp/testLocsAll.pdf", 20cm, 10cm),pl)

pl = drawLandms(fg)
# Gadfly.draw(PDF("/tmp/testAll.pdf", 20cm, 10cm),pl)
```

![testall](https://user-images.githubusercontent.com/6412556/42429216-d3b8b73a-8305-11e8-89ba-bb790b0963d5.png)

This example used the default of `N=200` particles per marginal belief.
By increasing the number to `N=300` throughout the test many more modes and interesting features can be explored, and we refer the reader to an alternative and longer discussion on the same example, in [Chapter 6 here](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1).
