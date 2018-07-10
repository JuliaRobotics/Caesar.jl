# Visualization with Arena.jl

Caesar.jl uses the [Arena.jl](https://github.com/dehann/Arena.jl) package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl.
Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.

Over time, Caesar.jl has used a least three different 3D visualization technologies, with the most recent based on WebGL and [three.js](https://threejs.org/) by means of the [MeshCat.jl](https://github.com/rdeits/MeshCat.jl) package.
The previous incarnation used a client side installation of [VTK](https://www.vtk.org/)  by means of the [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) and [Director](https://github.com/RobotLocomotion/director) libraries.
Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base.
Each epoch has aimed at reducing dependencies and increasing multi-platform support.

The sections below discuss 2D and 3D visualization techniques available to the Caesar.jl robot navigation system.
Visualization examples will be seen throughout the Caesar.jl package documentation.

**Note** that all visualizations used to be part of the Caesar.jl package itself, but was separated out to Arena.jl in early 2018.

## Installation

The current version of `Arena` has a rather large VTK dependency (which compile just fine on Ubuntu/Debian, or maybe even MacOS) wrapped in the [DrakeVisualizer.jl package](https://github.com/rdeits/DrakeVisualizer.jl).  This requires the following preinstalled packages:
```bash
    sudo apt-get install libvtk5-qt4-dev python-vtk
```

**NOTE** Smaller individual 2D packages can be installed instead -- i.e.:
```julia
Pkg.add("RoMEPlotting")
```

For the full 2D/3D visualization tools used by Caesar.jl---in a [Julia](http://www.julialang.org) or ([JuliaPro](http://www.juliacomputing.com)) terminal/REPL---type:
```julia
julia> Pkg.add("Arena")
```

**NOTE** Current development will allow the user to choose a `three.jl` WebGL based viewer instead [MeshCat.jl](https://github.com/rdeits/MeshCat.jl).

## 2D Visualization

2D plot visualizations, provided by `RoMEPlotting.jl` and `KernelDensityEstimatePlotting.jl`, are generally useful for repeated analysis of a algorithm or data set being studied.
These visualizations are often manipulated to emphasize particular aspects of mobile platform navigation.
Arena.jl is intended to simplify the process 2D plotting for robot trajectories in two or three dimensions.
The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets.

### Hexagonal 2D SLAM example visualization

The major 2D plotting functions between `RoMEPlotting.jl`:
- `drawPoses`
- `drawPosesLandms`
- `drawSubmaps`

and `KernelDensityEstimatePlotting.jl`:
- `plotKDE` / `plot(::KernelDensityEstimate)`


This simplest example for visualizing a 2D robot trajectory---such as first running [the Hexagonal 2D SLAM example](http://www.juliarobotics.org/Caesar.jl/latest/tut_hexagonal2d.html)---
```julia
# Assuming some fg::FactorGraph has been loaded/constructed
# ...

using RoMEPlotting

# For Juno/Jupyter style use
pl = drawPosesLandms(fg)

# For scripting use-cases you can export the image
Gadfly.draw(PDF("/tmp/test.pdf", 20cm, 10cm),pl)  # or PNG(...)
```

![test](https://user-images.githubusercontent.com/6412556/42294545-c6c80f70-7faf-11e8-8167-017889cee932.png)

## Density Contour Map

`KernelDensityEstimatePlotting` (as used in `RoMEPlotting`) provides an interface to visualize belief densities as counter plots.
The following basic example shows some of features of the API, where `plotKDE(..., dims=[1;2])` implies the marginal over variables `(x,y)`:

```julia
using RoME, Distributions
using KernelDensityEstimatePlotting

fg = initfg()
addNode!(fg, :x0, Pose2)
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(2), eye(2))))
addNode!(fg, :x1, Pose2)
addFactor!(fg, [:x0;:x1], Pose2Pose2(MvNormal(zeros(2), eye(2))))

ensureAllInitialized!(fg)

# plot one contour density
X0 = plotKDE(fg, :x1, dims=[1;2])
```



Multiple beliefs can be plotted at the same time


## 3D Visualization

Factor graphs of two or three dimensions can be visualized with the 3D visualizations provided by Arena.jl and it's dependencies.
The 2D example above and also be visualized in a 3D space with the commands:
```julia
vc = startdefaultvisualization() # to load a DrakeVisualizer/Director process instance
visualize(fg, vc, drawlandms=false)
# visualizeallposes!(vc, fg, drawlandms=false)
```  

Here is a basic example of using visualization and multi-core factor graph solving:
```julia
addprocs(2)
using Caesar, RoME, TransformUtils, Distributions

# load scene and ROV model (might experience UDP packet loss LCM buffer not set)
sc1 = loadmodel(:scene01); sc1(vc)
rovt = loadmodel(:rov); rovt(vc)

initCov = 0.001*eye(6); [initCov[i,i] = 0.00001 for i in 4:6];
odoCov = 0.0001*eye(6); [odoCov[i,i] = 0.00001 for i in 4:6];
rangecov, bearingcov = 3e-4, 2e-3

# start and add to a factor graph
fg = identitypose6fg(initCov=initCov)
tf = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf), odoCov) ) )

addLinearArrayConstraint(fg, (4.0, 0.0), :x0, :l1, rangecov=rangecov,bearingcov=bearingcov)
addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)

solveBatch!(fg)

using Arena

vc = startdefaultvisualization()
visualize(fg, vc, drawlandms=true, densitymeshes=[:l1;:x2])
visualizeDensityMesh!(vc, fg, :l1)
# visualizeallposes!(vc, fg, drawlandms=false)
```
