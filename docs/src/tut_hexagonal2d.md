# Tutorial: Hexagonal 2D Example (Local Compute)

A simple 2D robot trajectory example is:
```julia
# add more julia processes
nprocs() < 3 ? addprocs(4-nprocs()) : nothing

# tell Julia that you want to use these namespaces
using RoME, Distributions

# start with an empty factor graph object
fg = initfg()

# Add the first pose :x0
addNode!(fg, :x0, Pose2)
# Add at a fixed location PriorPose2 to pin :x0 to a starting location
addFactor!(fg, [:x0], PriorPose2(zeros(3,1), 0.01*eye(3), [1.0]))

# Drive around in a hexagon
for i in 0:5
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  addNode!(fg, nsym, Pose2)
  pp = Pose2Pose2(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))
  addFactor!(fg, [psym;nsym], pp )
end

# perform inference, and remember first runs are slower owing to Julia's just-in-time compiling
batchSolve!(fg)


## Inter-operating visualization packages for Caesar/RoME/IncrementalInference exist
using RoMEPlotting

# For Juno/Jupyter style use
pl = drawPoses(fg)

# For scripting use-cases you can export the image
Gadfly.draw(Gadfly.PDF("/tmp/test.pdf", 20cm, 10cm),pl)  # or PNG(...)
```

![test](https://user-images.githubusercontent.com/6412556/42294545-c6c80f70-7faf-11e8-8167-017889cee932.png)

See further [discussion on visualizations and packages here](http://www.juliarobotics.org/Caesar.jl/latest/arena_visualizations.html).
