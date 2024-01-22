# Known Issues

This page is used to list known issues:

- Arena.jl is fairly behind on a number of updates and deprecations.  Fixes for this are planned 2021Q2.
- RoMEPlotting.jl main features like `plotSLAM2D` are working, but some of the other features are not fully up to date with recent changes in upstream packages.  This too will be updated around Summer 2021.

## Features To Be Restored

### Install 3D Visualization Utils (e.g. Arena.jl)

3D Visualizations are provided by [Arena.jl](https://github.com/JuliaRobotics/Arena.jl) as well as development package Amphitheater.jl.
Please follow instructions on the [Visualizations page](../concepts/arena_visualizations.md) for a variety of 3D utilities.

!!! note
    Arena.jl and Amphitheater.jl are currently being refactored as part of the broader DistributedFactorGraph migration, the features are are in beta stage (1Q2020).

Install the latest `master` branch version with
```julia
(v1.5) pkg> add Arena#master
```

## Install "Just the ZMQ/ROS Runtime Solver" (Linux)

Work in progress (see issue [#278](https://github.com/JuliaRobotics/Caesar.jl/issues/278)).
