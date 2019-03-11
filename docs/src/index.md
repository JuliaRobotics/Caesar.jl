```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png" width="480" border="0" />
</p>
```

## Introduction
Caesar is a modern robotic framework for localization and mapping, reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM). Caesar attempts to address a number of issues that arise in normal SLAM solutions - solving under-defined systems, inference with non-Gaussian measurement distributions, simplifying factor creation, and centralizing factor-graph persistence with databases. Caesar started as part of the thesis "Multi-modal and Inertial Sensor Solutions for Navigation-type Factor Graphs" [[1]](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1).

## Focus Area

This project focuses on the open development and progression to a public, stable, growing and usable inference library suited to data-fusion aspects of device navigation.

## TL;DR Installation

### Install Inference Tools

Linux system dependencies are:
```bash
sudo apt-get install hdf5-tools
sudo apt-get install graphviz  # optional
```

Add Caesar to your Julia packages, you can install the metadata registered package 'Caesar' in Julia 1.0 with:
```julia
julia> ] # to enable package manager
(v1.0) pkg> add Caesar
```

Unit tests can further be performed for the upstream packages as follows -- **NOTE** first time runs are slow since each new function call or package must first be precompiled.
```julia
# the multimodal incremental smoothing and mapping solver
(v1.0) pkg> test IncrementalInference
...
# robotics related variables and factors to work with IncrementalInference -- can be used standalone SLAM system
(v1.0) pkg> test RoME
...
# umbrella framework with interaction tools and more -- allows stand alone and server based solving
(v1.0) pkg> test Caesar
...
```

### Install Visualization Tools

(Q4 2018), Temporarily require development (`master` branch) version of RoMEPlotting.jl (2D) and Arena.jl (3D) as optional visualization packages:
```julia
(v1.0) pkg> add RoMEPlotting#master

# separately
(v1.0) pkg> add Arena#master
```

## Caesar Framework

## Caesar Core
Caesar is implemented in [Julia](http://www.julialang.org/) (and [JuliaPro](http://www.juliacomputing.com)) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages as listed in features below.

[Caesar.jl](http://www.github.com/JuliaRobotics/Caesar.jl) is the "umbrella" framework that depends on and supports various dedicated purpose packages.

> **Why Julia**: The implementation language of choice is Julia ([but not limited to](http://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/)), for a wide variety of reasons such as agile development along side modern, high speed, type safe, multi-processor, JIT-dynamic, cross compiling (gcc and clang) and cross-platform technologies -- also see [JuliaCon2018 highlights video](https://www.youtube.com/watch?v=baR02tlea5Y).  Julia can be thought of as either {C+, Mex (done right), or a Fortran replacement}.  The Caesar.jl project is expressly focussed on making this algorithmic code available to C/C++/C#/Python/Java/JS/ applications through a variety of interfaces described below.  Please open issues or get in touch for more information about interops.

### Caesar Core Packages
Critically, this package can operate in the conventional SLAM manner, using local memory (dictionaries), or alternatively distribute around a persisted `FactorGraph` through a graph database using [CloudGraphs.jl](https://github.com/GearsAD/CloudGraphs.jl.git), as [discussed in literature here](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf) [2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) and back-end solver [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl).

Details about the accompanying packages:
* [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl) supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself.
* [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) introduces nodes and factors that are useful to robotic navigation.

### Visualization

Caesar visualization (plotting of results, graphs, and data) is provided by 2D and 3D packages respectively:
* [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D;
* [Arena.jl](https://github.com/dehann/Arena.jl) package, which is a collection of 3D visualization tools.

## Multilanguage Interops: Caesar SDKs and APIs
The Caesar framework is not limited to direct Julia use. The following Github projects provide access to features of Caesar in their language:

* Julia Web interface:
  * [GraffSDK.jl](https://github.com/GearsAD/GraffSDK.jl)
* C/C++:
  * [Graff Cpp](https://github.com/MarineRoboticsGroup/graff_cpp)
  * [Caesar LCM](http://github.com/pvazteixeira/caesar-lcm)
* Python:
  * [SynchronySDK](https://github.com/nicrip/SynchronySDK_py)

Contributions are welcome! If you are developing an extension we would like to help, please feel free to contact us (details below).

## Features
The Caesar framework has the following features:
* Factor-graph representation of pose and sensor data
* Localization using [Multi-modal iSAM](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Related-Literature-1)
  * Multi-core inference supporting `Pose2, Pose3, Point2, Point3, Multi-modal (multi-hypothesis), IMU preintegration, KDE density, intensity map, partial constraints, null hypothesis, etc`
* Multi-modal and non-parametric representation of constraints
  * Gaussian distributions are but one of the many representations of measurement error
  * Simple, extensible framework for creation of new factor types
* Multi-hypothesis representation in the factor-graph
* Local in-memory solving on the device as well as database-driven centralized solving
* Fixed-lag, continuous operation as well as off-line batch solving

## Origins in Fundamental Research

See related works on [the literature page](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/).

## Future Directions

Many future directions are in the works -- including fundamental research, implementation quality/performance, and system integration.  Please see/open issues for specific requests or adding comments to an ongoing discussion.  For example see [ROS integration page here](https://github.com/JuliaRobotics/Caesar.jl/issues/227).

## Next Steps
For installation steps, examples/tutorials, and concepts please refer to the following pages:

```@contents
Pages = [
    "installation_environment.md"
    "concepts/concepts.md"
    "examples/examples.md"
    "func_ref.md"
]
Depth = 3
```

## Future
This package is a work in progress. Please file issues here as needed to help resolve problems for everyone! We are tracking improvements and new endeavors in the Issues section of this repository.

In the future, Caesar will likely interact more closely with repos such as:
* [SensorFeatureTracking.jl](http://www.github.com/JuliaRobotics/SensorFeatureTracking.jl)
* [AprilTags.jl](http://www.github.com/JuliaRobotics/AprilTags.jl)
* [RecursiveFiltering.jl](http://www.github.com/JuliaRobotics/RecursiveFiltering.jl)

## JuliaRobotics Code of Conduct
The Caesar repository is part of the JuliaRobotics organization and adheres to the JuliaRobotics [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).


## Contributors

We are grateful for many, many contributions within the Julia package ecosystem -- see the `REQUIRE` files of `Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs` and others for a far reaching list of contributions.

Consider citing our work:

```
@misc{caesarjl,
  author = "Caesar.jl Contributors",
  title =  "Caesar.jl",
  year =   2018,
  url =    "https://github.com/JuliaRobotics/Caesar.jl"
}
```

Administration of the Caesar/RoME/IncrementalInference/Arena packages is currently conducted by Dehann Fourie who can be contacted for more details.
