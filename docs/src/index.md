```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png" width="480" border="0" />
</p>
```

[![Caesar](http://pkg.julialang.org/badges/Caesar_0.7.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.7)

# Introduction
Caesar is a modern robotic framework for localization and mapping, reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM). Caesar attempts to address a number of issues that arise in normal SLAM solutions - solving under-defined systems, inference with non-gaussian measurement distributions, simplifying factor creation, and centralizing factor-graph persistence with databases. Caesar started as part of the thesis "Towards non-parametric / parametric state estimation and navigation solutions" [[1]](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1).

## Features
The Caesar framework has the following features:
* Factor-graph representation of pose and sensor data
* Localization using [Multi-modal iSAM](http://frc.ri.cmu.edu/~kaess/pub/Fourie16iros.pdf)
  * Multi-core inference supporting `Pose2, Pose3, Point2, Point3, Null hypothesis, Multi-modal, KDE density, partial constraints`
* Multi-modal and non-parametric representation of constraints
  * Gaussian distributions are but one of the many representations of measurement error
  * Simple, extensible framework for creation of new factor types
* Multi-hypothesis representation in the factor-graph
* Local in-memory solving on the device as well as database-driven centralized solving
* Fixed-lag, continuous operation as well as off-line batch solving

## TLDR Installation
If you want to skip ahead and add Caesar to your Julia packages, you can install the metadata registered package 'Caesar'.

In a Julia 0.7+ REPL, press '?' and type `add Caesar` to pull the latest tagged version.

## Caesar Framework

## Caesar Core
Caesar is implemented in [Julia](http://www.julialang.org/) (and [JuliaPro](http://www.juliacomputing.com)) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages as listed in features below.

[Caesar.jl](http://www.github.com/JuliaRobotics/Caesar.jl) is the umbrella repo that depends on RoME.jl and others to support that 'passes through' the same functionality while introducing more. For example, interaction with database server systems, [LCMCore.jl](http://www.github.com/JuliaRobotics/LCMCore.jl), (future ROS support), and more.

### Caesar Core Packages
Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the `FactorGraph` through a graph database using [CloudGraphs.jl](https://github.com/GearsAD/CloudGraphs.jl.git), as [discussed here](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf) [2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) and back-end solver [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl).

Details about the accompanying packages:
* [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl) supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself.
* [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) introduces nodes and factors that are useful to robotic navigation.
* [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D.

## Caesar Extensions

### Visualization
Caesar visualization (plotting of results, graphs, and data) is provided in the [Arena.jl](https://github.com/dehann/Arena.jl) package, which is a collection of 3D visualization tools and also depends on RoMEPlotting.jl for 2D visualizations.

## Caesar SDKs and APIs
The Caesar framework is not limited to direct Julia use. The following Github projects provide access to features of Caesar in their language:

* C/C++:
  * [Graff Cpp](https://github.com/MarineRoboticsGroup/graff_cpp)
  * [Caesar LCM](http://github.com/pvazteixeira/caesar-lcm)
* Python:
  * [SynchronySDK](https://github.com/nicrip/SynchronySDK_py)

Contributions are welcome! If you are developing an extension we would like to help, please feel free to contact us (details below).

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
Authors directly involved with this package are:

D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. Leonard

We are grateful for many, many contributions within the Julia package ecosystem -- see the `REQUIRE` files of `Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs` and others for a far reaching list of contributions.


## Cite
Consider citing our work:

```
@misc{caesarjl,
  author = "Dehann Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, John Leonard, Micheal Kaess, and other contributors",
  title =  "Caesar.jl",
  year =   2018,
  url =    "https://github.com/JuliaRobotics/Caesar.jl"
}
```

# References
    [1]  Fourie, D.: "Multi-modal and Inertial Sensor Solutions to Navigation-type Factor Graph",
         Ph.D. Thesis, Massachusetts Institute of Technology Electrical Engineering and Computer Science together with Woods Hole Oceanographic Institution Department for Applied Ocean Science and Engineering, September 2017.
    [2]  Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: "SLAMinDB: Centralized graph
         databases for mobile robotics" IEEE International Conference on Robotics and Automation (ICRA),
         Singapore, 2017.
