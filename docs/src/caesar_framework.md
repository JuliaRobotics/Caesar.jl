
# The Caesar Framework

The [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) package is an "umbrella" framework around other dedicated algorithmic packages.  While most of the packages are implemented in native [Julia](http://www.julialang.org/) ([JuliaPro](http://www.juliacomputing.com)), a few dependencies are wrapped C libraries.  Note that C/C++ [can be incorporated with zero overhead](https://docs.julialang.org/en/v1/manual/calling-c-and-fortran-code/), such as was done with [AprilTags.jl](http://www.github.com/JuliaRobotics/AprilTags.jl).

> [FAQ: Why use Julia?](https://www.juliarobotics.org/Caesar.jl/latest/faq/#Why-Julia-1)

## AMP / IIF / RoME

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

## Multilanguage Interops: NavAbility.io SDKs and APIs

The Caesar framework is not limited to direct Julia use.  Check out www.NavAbility.io, or contact directly at (info@navabiliyt.io), for more details.    Also see the [community multi-language page](https://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/) for details.


!!! note
    [FAQ: Interop with other languages (not limited to Julia only)](https://www.juliarobotics.org/Caesar.jl/latest/faq/#Is-Caesar.jl-limited-to-Julia?-No.-1)

