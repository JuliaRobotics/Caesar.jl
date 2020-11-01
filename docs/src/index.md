```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png" width="480" border="0" />
</p>
```

## Introduction
Caesar is an open-source robotic software stack for combining heterogeneous and ambiguous data streams.  The focus is predominantly on geometric/spatial estimation tasks related to simultaneous localization and mapping (SLAM), but this software is also highly extensible and well suited to a variety of estimation/filtering-type tasks — especially in non-Gaussian/multimodal settings.  Caesar.jl addresses numerous issues that arise in prior SLAM solutions: solving under-defined systems, inference with non-Gaussian measurements, standard features for natively handling ambiguous data association and multi-hypotheses, simplifying bespoke factor development, centralized (or peer-to-peer distributed) factor-graph persistence with databases and cloud infrastructure, federated multi-session/agent reduction.  Caesar.jl originates from research work in navigation systems, see the [literature reference page](https://www.juliarobotics.org/Caesar.jl/latest/refs/literature/) for more information.

[NavAbility](https://www.navability.io/) administers the Caesar/RoME/IncrementalInference packages, please contact info@navability.io for more details.

## Focus Area

This project focuses on the open development of a stable, reliable, verified, user-friendly, and growing library that is well suited to various data-fusion / state-estimation aspects of robotics and autonomy in [non-Gaussian/multi-modal](https://juliarobotics.org/Caesar.jl/latest/concepts/concepts/#Why/Where-does-non-Gaussian-data-come-from?-1) data processing.

## A Few Highlights

The Caesar framework offers these and other features:
* Distributed Factor Graph representation deeply-coupled with an on-Manifold probabilistic algebra language;
* Localization using [MM-iSAMv2](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Related-Literature-1)
  * Multi-core inference supporting `Pose2, Pose3, Point2, Point3, Multi-modal (multi-hypothesis), IMU preintegration, KDE density, intensity map, partial constraints, null hypothesis, etc`.
* Native multi-modal (hypothesis) representation in the factor-graph,
  * Multi-modal and non-parametric representation of constraints;
  * Gaussian distributions are but one of the many representations of measurement error;
  * Simple, extensible framework for creation of new factor types.
* Natively supports legacy Gaussian parametric and max-mixtures solutions;
* Local in-memory solving on the device as well as database-driven centralized solving (micro-service architecture);
* Natively support Fixed-lag (out-marginalization) for continuous operation as well as off-line batch solving;
* Natively supports Dead Reckon Tethering;
* Natively supports Federated multi-session/agent solving;
* Native support for `Entry=>Data` blobs [for storing large format data](https://juliarobotics.org/Caesar.jl/latest/concepts/entry_data/).

# The Caesar Framework

The [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) package is an "umbrella" framework around other dedicated algorithmic packages.  While most of the packages are implemented in native [Julia](http://www.julialang.org/) ([JuliaPro](http://www.juliacomputing.com)), a few dependencies are wrapped C libraries.  Note that C/C++ [can be incorporated with zero overhead](https://docs.julialang.org/en/v1/manual/calling-c-and-fortran-code/), such as was done with [AprilTags.jl](http://www.github.com/JuliaRobotics/AprilTags.jl).

> [FAQ: Why use Julia?](https://www.juliarobotics.org/Caesar.jl/latest/faq/#Why-Julia-1)

## RoME.jl/IncrementalInference.jl/ApproxManifoldProducts.jl

Robot motion estimate ([RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl)) can operate in the conventional SLAM manner, using local memory (dictionaries), or alternatively distribute over a persisted [`DistributedFactorGraph.jl`](http://www.github.com/JuliaRobotics/DistributedFactorGraphs.jl) through common serialization and graph storage/database technologies, [see this article as example](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf) [1.3].  
A variety of 2D plotting, 3D visualization, serialization, middleware, and analysis tools come standard as provided by the associated packages.  RoME.jl combines reference frame transformations and robotics SLAM tool around the back-end solver provides by [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl).

Details about the accompanying packages:
* [IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl) supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself.
* [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) introduces nodes and factors that are useful to robotic navigation.
* [ApproxManifoldProducts.jl](http://www.github.com/JuliaRobotics/ApproxManifoldProducts.jl) provides on-manifold belief product operations.

## Visualization (Arena.jl/RoMEPlotting.jl)
Caesar visualization (plotting of results, graphs, and data) is provided by 2D and 3D packages respectively:
* [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D;
* [Arena.jl](https://github.com/dehann/Arena.jl) package, which is a collection of 3D visualization tools.

## Multilanguage Interops: Caesar SDKs and APIs
The Caesar framework is not limited to direct Julia use.  See the [multi-language page](https://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/) for details.

> [FAQ: Interop with other languages (not limited to Julia only)](https://www.juliarobotics.org/Caesar.jl/latest/faq/#Is-Caesar.jl-limited-to-Julia?-No.-1)

## Origins in Fundamental Research

See related works on [the literature page](https://www.juliarobotics.org/Caesar.jl/latest/refs/literature/).  Many future directions are in the works -- including fundamental research, implementation quality/performance, and system integration.  Please see/open issues for specific requests or adding comments to an ongoing discussion -- also consult the Caesar.jl Slack channel to follow/engage with community discussions.

!!! note
    Please help improve this documentation--if something confuses you, chances
    are you're not alone. It's easy to do as you read along: just click on the
    "Edit on GitHub" link above, and then
    [edit the files directly in your browser](https://help.github.com/articles/editing-files-in-another-user-s-repository/).
    Your changes will be vetted by developers before becoming permanent, so don't
    worry about whether you might say something wrong.

## Contributors

We are grateful for many, many contributions within the Julia package ecosystem -- see the `Project.toml` files of `Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, LightGraphs, Gadfly, MeshCat` and others for a far reaching list of contributions.

Consider citing our work:
```
@misc{caesarjl,
  author = "Contributors and Packages",
  title =  "Caesar.jl",
  year =   2020,
  url =    "https://github.com/JuliaRobotics/Caesar.jl"
}
```

Please file issues via Github for help to resolve problems for everyone. Issues can be transfered moved upstream to the best suited package location. Changes and ongoing work can be tracked via the [Milestone pages (click through badges here)](https://github.com/JuliaRobotics/Caesar.jl/blob/master/README.md#bleeding-edge-development-status). 

## JuliaRobotics Code of Conduct
The Caesar.jl project is part of the JuliaRobotics organization and adheres to the JuliaRobotics [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).

# Next Steps
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

