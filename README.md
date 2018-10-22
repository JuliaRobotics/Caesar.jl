<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png" width="480" border="0" />
</p>

A modern robotic toolkit for localization and mapping -- reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM).

[![Build Status][build-img]][build-url]
<!--
[![Caesar](http://pkg.julialang.org/badges/Caesar_0.6.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.6)-->

# Introduction

Towards non-parametric / parametric state estimation and navigation solutions [1]. Implemented in [Julia](http://www.julialang.org/) (and [JuliaPro](http://www.juliacomputing.com)) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages like C/[C++](http://github.com/pvazteixeira/caesar-lcm) or [Python](http://github.com/JuliaRobotics/Caesar.jl/blob/master/examples/database/python/neo4j_interact_example.py), as listed in features below. Multi-modal (quasi-multi-hypothesis) navigation and mapping solutions, using various sensor data, is a corner stone of this package. Multi-sensor fusion is made possible via vertically integrated [Multi-modal iSAM](http://frc.ri.cmu.edu/~kaess/pub/Fourie16iros.pdf).

Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the `FactorGraph` through a graph database using [CloudGraphs.jl](https://github.com/GearsAD/CloudGraphs.jl.git), as [discussed here](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf)[2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl][rome-url] and back-end solver [IncrementalInference.jl][iif-url].

Comments, questions and issues welcome.

# Documentation

Please see the [documentation](http://juliarobotics.github.io/Caesar.jl/latest/):

[![docs](https://img.shields.io/badge/docs-latest-blue.svg)](http://juliarobotics.github.io/Caesar.jl/latest/)

# Visualization

Please see the [Arena.jl](http://www.github.com/JuliaRobotics/Arena.jl) package for the concentration of all 2D and 3D visualization utilities of the Caesar.jl robot navigation packages.

# Installation

Caesar can be installed with:
```julia
julia> Pkg.add("Caesar")
```

# Bleeding-edge Development Status

| **Major Dependencies** |     **Status**     |    **Test Coverage**    |
|:-----------------------:|:------------------:|:------------------:|
| Caesar.jl | [![Build Status][build-img]][build-url] | [![codecov.io][cov-img]][cov-url] |
| [RoME.jl][rome-url] | [![Build Status][r-build-img]][r-build-url] | [![codecov.io][r-cov-img]][r-cov-url] |
| [IncrementalInference.jl][iif-url] | [![Build Status][iif-build-img]][iif-build-url] | [![codecov.io][iif-cov-img]][iif-cov-url] |
| [KernelDensityEstimate.jl][kde-url] | [![Build Status][kde-build-img]][kde-build-url] | [![codecov.io][kde-cov-img]][kde-cov-url] |
| [TransformUtils.jl][tf-url] | [![Build Status][tf-build-img]][tf-build-url] | [![codecov.io][tf-cov-img]][tf-cov-url] |
| [Graphs.jl][graphs-url] | [![Build Status][graphs-build-img]][graphs-build-url] | [![codecov.io][graphs-cov-img]][graphs-cov-url] |
| [CloudGraphs.jl][cloudgraphs-url] | [![Build Status][cloudgraphs-build-img]][cloudgraphs-build-url] | [![codecov.io][cloudgraphs-cov-img]][cloudgraphs-cov-url] |

# Contributors

Authors directly involved with this package are:

D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. Leonard

We are grateful for many, many contributions within the Julia package ecosystem -- see the `REQUIRE` files of `Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs` and others for a far reaching list of contributions.

# Cite

Consider citing our work:

```
@misc{caesarjl,
  author = "Dehann Fourie, John Leonard, Micheal Kaess, and contributors",
  title =  "Caesar.jl",
  year =   2017,
  url =    "https://github.com/JuliaRobotics/Caesar.jl"
}
```


[cov-img]: https://codecov.io/github/JuliaRobotics/Caesar.jl/coverage.svg?branch=master
[cov-url]: https://codecov.io/github/JuliaRobotics/Caesar.jl?branch=master
[build-img]: https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=master
[build-url]: https://travis-ci.org/JuliaRobotics/Caesar.jl

[rome-url]: http://www.github.com/JuliaRobotics/RoME.jl
[r-cov-img]: https://codecov.io/github/JuliaRobotics/RoME.jl/coverage.svg?branch=master
[r-cov-url]: https://codecov.io/github/JuliaRobotics/RoME.jl?branch=master
[r-build-img]: https://travis-ci.org/JuliaRobotics/RoME.jl.svg?branch=master
[r-build-url]: https://travis-ci.org/JuliaRobotics/RoME.jl

[iif-cov-img]: https://codecov.io/github/JuliaRobotics/IncrementalInference.jl/coverage.svg?branch=master
[iif-cov-url]: https://codecov.io/github/JuliaRobotics/IncrementalInference.jl?branch=master
[iif-build-img]: https://travis-ci.org/JuliaRobotics/IncrementalInference.jl.svg?branch=master
[iif-build-url]: https://travis-ci.org/JuliaRobotics/IncrementalInference.jl
[iif-url]: http://www.github.com/JuliaRobotics/IncrementalInference.jl

[kde-cov-img]: https://codecov.io/github/JuliaRobotics/KernelDensityEstimate.jl/coverage.svg?branch=master
[kde-cov-url]: https://codecov.io/github/JuliaRobotics/KernelDensityEstimate.jl?branch=master
[kde-build-img]: https://travis-ci.org/JuliaRobotics/KernelDensityEstimate.jl.svg?branch=master
[kde-build-url]: https://travis-ci.org/JuliaRobotics/KernelDensityEstimate.jl
[kde-url]: http://www.github.com/JuliaRobotics/KernelDensityEstimate.jl

[tf-cov-img]: https://codecov.io/github/dehann/TransformUtils.jl/coverage.svg?branch=master
[tf-cov-url]: https://codecov.io/github/dehann/TransformUtils.jl?branch=master
[tf-build-img]: https://travis-ci.org/dehann/TransformUtils.jl.svg?branch=master
[tf-build-url]: https://travis-ci.org/dehann/TransformUtils.jl
[tf-url]: http://www.github.com/dehann/TransformUtils.jl

<!-- | [DrakeVisualizer.jl][dvis-url] | [![Build Status][dvis-build-img]][dvis-build-url] | [![codecov.io][dvis-cov-img]][dvis-cov-url] |
[dvis-cov-img]: https://codecov.io/github/rdeits/DrakeVisualizer.jl/coverage.svg?branch=master
[dvis-cov-url]: https://codecov.io/github/rdeits/DrakeVisualizer.jl?branch=master
[dvis-build-img]: https://travis-ci.org/rdeits/DrakeVisualizer.jl.svg?branch=master
[dvis-build-url]: https://travis-ci.org/rdeits/DrakeVisualizer.jl
[dvis-url]: http://www.github.com/rdeits/DrakeVisualizer.jl -->

[graphs-cov-img]: https://codecov.io/github/JuliaAttic/Graphs.jl/coverage.svg?branch=master
[graphs-cov-url]: https://codecov.io/github/JuliaAttic/Graphs.jl?branch=master
[graphs-build-img]: https://travis-ci.org/JuliaAttic/Graphs.jl.svg?branch=master
[graphs-build-url]: https://travis-ci.org/JuliaAttic/Graphs.jl
[graphs-url]: http://www.github.com/JuliaAttic/Graphs.jl

[cloudgraphs-cov-img]: https://codecov.io/github/GearsAD/CloudGraphs.jl/coverage.svg?branch=master
[cloudgraphs-cov-url]: https://codecov.io/github/GearsAD/CloudGraphs.jl?branch=master
[cloudgraphs-build-img]: https://travis-ci.org/GearsAD/CloudGraphs.jl.svg?branch=master
[cloudgraphs-build-url]: https://travis-ci.org/GearsAD/CloudGraphs.jl
[cloudgraphs-url]: http://www.github.com/GearsAD/CloudGraphs.jl
