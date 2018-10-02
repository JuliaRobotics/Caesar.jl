```@raw html
<p align="center">
<img src="assets/logo.png" width="480" border="0" />
</p>
```
A modern robotic toolkit for localization and mapping -- reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM).

[![Caesar](http://pkg.julialang.org/badges/Caesar_0.6.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.6)

Towards non-parametric / parametric state estimation and navigation solutions [1]. Implemented in [Julia](http://www.julialang.org/) (and [JuliaPro](http://www.juliacomputing.com)) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages like C/[C++](http://github.com/pvazteixeira/caesar-lcm) or [Python](http://github.com/dehann/Caesar.jl/blob/master/examples/database/python/neo4j_interact_example.py), as listed in features below. Multi-modal (quasi-multi-hypothesis) navigation and mapping solutions, using various sensor data, is a corner stone of this package. Multi-sensor fusion is made possible via vertically integrated [Multi-modal iSAM](http://frc.ri.cmu.edu/~kaess/pub/Fourie16iros.pdf).

Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the `FactorGraph` through a graph database using [CloudGraphs.jl](https://github.com/GearsAD/CloudGraphs.jl.git), as [discussed here](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf)[2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl][rome-url] and back-end solver [IncrementalInference.jl][iif-url].

Comments, questions and issues welcome.

# Dependency Structure

[IncrementalInference.jl](http://www.github.com/JuliaRobotics/IncrementalInference.jl) supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself. [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl) introduces nodes and factors that are useful to robotic navigation. [RoMEPlotting.jl](http://www.github.com/JuliaRobotics/RoMEPlotting.jl) are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D.

[Caesar.jl](http://www.github.com/JuliaRobotics/Caesar.jl) is the umbrella repo that depends on RoME.jl and others to support that 'passes through' the same functionality while introducing more. For example, interaction with database server systems, [LCMCore.jl](http://www.github.com/JuliaRobotics/LCMCore.jl), (future ROS support), and more.

[Arena.jl](http://www.github.com/JuliaRobotics/Arena.jl) is a collection of 3D visualization tools and also depends on RoMEPlotting.jl for 2D visualizations.


In the future, Caesar.jl would likely interact more closely with repo's such as [SensorFeatureTracking.jl](http://www.github.com/JuliaRobotics/SensorFeatureTracking.jl), [AprilTags.jl](http://www.github.com/JuliaRobotics/AprilTags.jl), and [RecursiveFiltering.jl](http://www.github.com/JuliaRobotics/RecursiveFiltering.jl)

# Major features
---

* Performing multi-core inference with Multi-modal iSAM over factor graphs, supporting `Pose2, Pose3, Point2, Point3, Null hypothesis, Multi-modal, KDE density, partial constraints`, and more.
```julia
tree = wipeBuildBayesTree!(fg, drawpdf=true)
inferOverTree!(fg, tree)
```

* Or directcly on a database, allowing for separation of concerns
```julia
slamindb()
```

* Local copy of database held FactorGraph
```julia
fg = Caesar.initfg(cloudGraph, session)
fullLocalGraphCopy(fg)
```

* Saving and loading FactorGraph objects to file
```julia
savejld(fg, file="test.jld", groundtruth=gt)
loadjld(file="test.jld")
```

* Visualization through [Arena.jl](https://github.com/dehann/Arena.jl).
```julia
visualizeallposes(fg) # from local dictionary
drawdbdirector()      # from database held factor graph
```

* [Foveation queries](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf) to quickly organize, extract and work with big data blobs, for example looking at images from multiple sessions predicted to see the same point `[-9.0,9.0]` in the map:
```julia
neoids, syms = foveateQueryToPoint(cloudGraph,["SESS21";"SESS38";"SESS45"], "robot", "user" point=[-9.0;9.0], fovrad=0.5 )
for neoid in neoids
    cloudimshow(cloudGraph, neoid=neoid)
end
```

* Operating on data from a thin client processes, such as a Python front-end
 [examples/database/python/neo_interact_example.jl](https://github.com/dehann/Caesar.jl/blob/master/examples/database/python/neo4j_interact_example.py)

* A `caesar-lcm` server interface for C++ applications is [available here](http://github.com/pvazteixeira/caesar-lcm).

* A multicore Bayes 2D feature tracking server over tcp
```
julia -p10 -e "using Caesar; tcpStringBRTrackingServer()"
```

And many more, please see the [examples](/examples) folder.

## Installation
---

Caesar.jl is registered with the regular Julia METADATA and can be installed as follows:
```julia
julia> Pkg.add("Caesar")
```

Please note that visualizations have been moved to the [Arena.jl](https://github.com/dehann/Arena.jl) package and documentation can be found on the visualization page of this documentation.

## Basic usage
---

The basic example has been moved to the visualization page.


## Future targets
---

This is a work in progress package. Please file issues here as needed to help resolve problems for everyone!

Hybrid parametric and non-parametric optimization. Incrementalized update rules and properly marginalized 'forgetting' for sliding window type operation. We defined interprocess interface for multi-language front-end development.

# Contributors
---

Authors directly involved with this package are:

D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. Leonard

We are grateful for many, many contributions within the Julia package ecosystem -- see the `REQUIRE` files of `Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs` and others for a far reaching list of contributions.

# Cite
---

Consider citing our work:

```
@misc{caesarjl,
  author = "Dehann Fourie, John Leonard, Micheal Kaess, and contributors",
  title =  "Caesar.jl",
  year =   2017,
  url =    "https://github.com/dehann/Caesar.jl"
}
```

# References
---

    [1]  Fourie, D.: "Multi-modal and Inertial Sensor Solutions to Navigation-type Factor Graph",
         Ph.D. Thesis, Massachusetts Institute of Technology Electrical Engineering and Computer Science together with Woods Hole Oceanographic Institution Department for Applied Ocean Science and Engineering, September 2017.
    [2]  Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: "SLAMinDB: Centralized graph
         databases for mobile robotics" IEEE International Conference on Robotics and Automation (ICRA),
         Singapore, 2017.

## Manual Outline

```@contents
Pages = [
    "index.md"
    "examples.md"
    "func_ref.md"
]
Depth = 3
```
