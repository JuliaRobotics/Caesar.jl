```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png" width="480" border="0" />
</p>
```

## Introduction
Caesar is a modern robotic framework for localization and mapping, reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM). Caesar attempts to address a number of issues that arise in normal SLAM solutions - solving under-defined systems, inference with non-Gaussian measurement distributions, simplifying factor creation, and centralizing factor-graph persistence with databases. Caesar started as part of [the thesis "Multi-modal and Inertial Sensor Solutions for Navigation-type Factor Graphs" [1.2]](refs/literature).

## Focus Area

This project focuses on the open development and progression to a public, stable, growing and usable inference library suited to data-fusion aspects of device navigation.

## The Caesar Framework

### Caesar
[Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) is the "umbrella" framework for other dedicated algorithmic/supporting packages that are implemented in [Julia](http://www.julialang.org/) (and [JuliaPro](http://www.juliacomputing.com)).

> [FAQ: Why use Julia?](faq/#Why-Julia-1)

> [FAQ: Interop with other languages](faq/#Is-Caesar.jl-limited-to-Julia?-No.)

### RoME.jl/IncrementalInference.jl/ApproxManifoldProducts.jl
Critically, this package can operate in the conventional SLAM manner, using local memory (dictionaries), or alternatively distribute around a persisted `FactorGraph` through a graph database using [CloudGraphs.jl](https://github.com/GearsAD/CloudGraphs.jl.git), as [discussed in literature here](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf) [1.3]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) and back-end solver [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl).

Details about the accompanying packages:
* [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl) supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself.
* [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) introduces nodes and factors that are useful to robotic navigation.

### Visualization (Arena.jl/RoMEPlotting.jl)
Caesar visualization (plotting of results, graphs, and data) is provided by 2D and 3D packages respectively:
* [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D;
* [Arena.jl](https://github.com/dehann/Arena.jl) package, which is a collection of 3D visualization tools.

## Multilanguage Interops: Caesar SDKs and APIs
The Caesar framework is not limited to direct Julia use.  See the [multi-language page](concepts/multilang) for details.

> Also see [Frequently Asked Questions](faq) for more.

## A Few Highlights

> **Work In Progress**: (must be updated, 2019Q1)

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

See related works on [the literature page](refs/literature/).

## Future Directions

Many future directions are in the works -- including fundamental research, implementation quality/performance, and system integration.  Please see/open issues for specific requests or adding comments to an ongoing discussion.

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
  author = "Contributors",
  title =  "Caesar.jl",
  year =   2019,
  url =    "https://github.com/JuliaRobotics/Caesar.jl"
}
```

Administration of the Caesar/RoME/IncrementalInference/Arena packages is currently conducted by Dehann Fourie who can be contacted for more details.
