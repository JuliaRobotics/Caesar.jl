# Visualization with Arena.jl

The sections below discuss 3D visualization techniques available to the Caesar.jl robot navigation system.
Caesar.jl uses the [Arena.jl](https://github.com/dehann/Arena.jl) package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl.
Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.
The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets.

!!! warning

    Arena and Amphitheater are being upgraded as part of the broader migration to DistributedFactorGraphs (1Q20)

## Introduction

Over time, Caesar.jl/Arena.jl has used a least three different 3D visualization technologies, with the most recent based on WebGL and [three.js](https://threejs.org/) by means of the [MeshCat.jl](https://github.com/rdeits/MeshCat.jl) package.
The previous incarnation used a client side installation of [VTK](https://www.vtk.org/)  by means of the [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) and [Director](https://github.com/RobotLocomotion/director) libraries.
Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base.
Each epoch has been aimed at reducing dependencies and increasing multi-platform support.

!!! note

    See [installation page](https://juliarobotics.org/Caesar.jl/latest/installation_environment/#Install-Visualization-Tools-1) for instructions.

## 3D Visualization

!!! warning

    This section is out of date, see proof or concept examples in Amphitheater.jl (1Q20).

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

## Previous 3D Viewer (VTK / Director) -- no longer required

Previous versions used the much larger VTK based Director available via [DrakeVisualizer.jl package](https://github.com/rdeits/DrakeVisualizer.jl).  This requires the following preinstalled packages:
```bash
    sudo apt-get install libvtk5-qt4-dev python-vtk
```
