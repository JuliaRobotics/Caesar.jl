# Visualization with Arena.jl

Caesar.jl uses the [Arena.jl](https://github.com/dehann/Arena.jl) package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl.
Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.

Over time, Caesar.jl has used a least three different 3D visualization technologies, with the most recent based on WebGL and [three.js](https://threejs.org/) by means of the [MeshCat.jl](https://github.com/rdeits/MeshCat.jl) package.
The previous incarnation used a client side installation of [VTK](https://www.vtk.org/)  by means of the [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) and [Director](https://github.com/RobotLocomotion/director) libraries.
Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base.
Each epoch has aimed at reducing dependencies and increasing multi-platform support.

The sections below discuss 2D and 3D visualization techniques available to the Caesar.jl robot navigation system.
Visualization examples will be seen throughout the Caesar.jl package documentation.

**Note** that all visualizations used to be part of the Caesar.jl package itself, but was separated out to Arena.jl in early.
There may be some package documentation glitches where the `using Arena` dependency has not been added -- please file issues or suggest changes accordingly.

## Installation

Requires via ```sudo apt-get install```, see [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) for more details.

    libvtk5-qt4-dev python-vtk


## 2D Visualization

2D plot visualizations are generally useful for repeated analysis of a algorithm or data set being studied.
These visualizations are often manipulated to emphasize particular aspects of mobile platform navigation.
Arena.jl is intended to simplify the process 2D plotting for robot trajectories in two or three dimensions.
The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets.

### Hexagonal 2D SLAM example visualization

This simplest example for visualizing a 2D robot trajectory is:
```julia
using RoME, Arena  

fg = initfg()

# also add a PriorPose2 to pin the first pose at a fixed location
addNode!(fg, :x0, Pose2, labels=["VARIABLE";"POSE"])
addFactor!(fg, [:x0], PriorPose2(zeros(3,1), 0.01*eye(3), [1.0]))

# Drive around in a hexagon
for i in 0:5
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  addNode!(fg, nsym, Pose2, labels=["VARIABLE";"POSE"])
  addFactor!(fg, [psym;nsym], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )
  # Pose2Pose2_NEW(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))
end

ensureAllInitialized!(fg)
solveBatch!(fg)

# The RoME and IncrementalInference

# RoMEPlotting, KernelDensityEstimatePlotting and Gadfly packages provide the 2D visualization
drawPoses(fg)
```

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
