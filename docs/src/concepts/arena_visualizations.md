# [Visualization 3D](@id visualization_3d)

## Introduction

Over time, Caesar.jl/Arena.jl has used at various different 3D visualization technologies.  

### Arena.jl Visualization

#### [Plotting a PointCloud](@id viz_pointcloud)

Visualization support for point clouds is available through Arena and Caesar.  The follow example shows some of the basics:

```julia
using Arena
using Caesar
using Downloads
using DelimitedFiles
using LasIO
using Test

##

function downloadTestData(datafile, url)
  if 0 === Base.filesize(datafile)
    Base.mkpath(dirname(datafile))
    @info "Downloading $url"
    Downloads.download(url, datafile)
  end
  return datafile
end

testdatafolder = joinpath(tempdir(), "caesar", "testdata") # "/tmp/caesar/testdata/"

lidar_terr1_file = joinpath(testdatafolder,"lidar","simpleICP","terrestrial_lidar1.xyz")
if !isfile(lidar_terr1_file)
  lidar_terr1_url = "https://github.com/JuliaRobotics/CaesarTestData.jl/raw/main/data/lidar/simpleICP/terrestrial_lidar1.xyz"
  downloadTestData(lidar_terr1_file,lidar_terr1_url)
end

# load the data to memory
X_fix = readdlm(lidar_terr1_file, Float32)
# convert data to PCL types
pc_fix = Caesar._PCL.PointCloud(X_fix);


pl = Arena.plotPointCloud(pc_fix)

```

This should result in a plot similar to:

```@raw html
<p align="center">
<img src="https://github.com/JuliaRobotics/Arena.jl/assets/6412556/7679c346-fbeb-4d84-b20d-13bd589f3338" width="800" border="0" />
</p>
```


!!! note
    24Q1: Currently work is underway to better standardize within the Julia ecosystem, with the 4th generation of Arena.jl -- note that this is work in progress.  Information about legacy generations is included below.

For more formal visualization support, contact [www.NavAbility.io](http://www.navability.io) via email or slack. 
## 4th Generation Dev Scripts using Makie.jl

Working towards new [Makie.jl](https://github.com/JuliaPlots/Makie.jl).  Makie supports both GL and WGL, including 3rd party libraries such as [three.js](https://threejs.org/) (previously used via MeshCat.jl, see Legacy section below.).
### [Visualizing Point Clouds](@id viz_pointcloud_makie)

Point clouds could be massive, on the order of a million points or more.  Makie.jl has good performance for handling such large point cloud datasets.  Here is a quick example script.
```julia
using Makie, GLMakie

# n x 3 matrix of 3D points in pointcloud
pts1 = randn(100,3)
pts2 = randn(100,3)

# plot first and update with second
plt = scatter(pts1[:,1],pts1[:,2],pts1[:,3], color=pts1[:,3])
scatter!(pts2[:,1],pts2[:,2],pts2[:,3], color=-pts2[:,3])
```
## Visualizing with Arena.jl

!!! warning
    Arena.jl is currently out of date since the package will likely support Makie via both GL and WGL interfaces.  Makie.jl has been receiving much attention over the past years and starting to mature to a point where Arena.jl can be revived again.  2D plotting is done via [`RoMEPlotting.jl`](@ref plotting_2d).

The sections below discuss 3D visualization techniques available to the Caesar.jl robot navigation system.
Caesar.jl uses the [Arena.jl](https://github.com/dehann/Arena.jl) package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl.
Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.
The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets.

## Legacy Visualizers

Previous generations used various technologies, including WebGL and [three.js](https://threejs.org/) by means of the [MeshCat.jl](https://github.com/rdeits/MeshCat.jl) package.
Previous incarnations used a client side installation of [VTK](https://www.vtk.org/)  by means of the [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) and [Director](https://github.com/RobotLocomotion/director) libraries.
Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base.
Each epoch has been aimed at reducing dependencies and increasing multi-platform support.

### 3rd Generation MeshCat.jl (Three.js)


For the latest work on using MeshCat.jl, see proof or concept examples in Amphitheater.jl (1Q20).  The code below inspired the Amphitheater work.

!!! note
    See [installation page](https://juliarobotics.org/Caesar.jl/latest/installation_environment/#Install-Visualization-Tools-1) for instructions.

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

For more information see [JuliaRobotcs/MeshCat.jl](https://github.com/rdeits/MeshCat.jl).

### 2nd Generation 3D Viewer (VTK / Director)

!!! note
    This code is obsolete

Previous versions used the much larger VTK based Director available via [DrakeVisualizer.jl package](https://github.com/rdeits/DrakeVisualizer.jl).  This requires the following preinstalled packages:
```bash
    sudo apt-get install libvtk5-qt4-dev python-vtk
```

### 1st Generation MIT LCM Collections viewer

This code has been removed.
