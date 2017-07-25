<p align="center">
<img src="https://raw.githubusercontent.com/dehann/Caesar.jl/master/doc/imgs/julia_caesar 001_eyes_thres_wtitle_smaller_centered_low.png" width="480" border="0" />
</p>

A modern robotic toolkit for localization and mapping -- reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM).

[![Build Status][build-img]][build-url]
<!-- [![Caesar](http://pkg.julialang.org/badges/Caesar_0.5.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.5)
[![Caesar](http://pkg.julialang.org/badges/Caesar_0.6.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.6)-->

Towards non-parametric / parametric state estimation and navigation solutions. Implemented in [Julia](http://www.julialang.org/) (and [JuliaPro](http://www.juliacomputing.com)) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages like C/[C++](http://github.com/pvazteixeira/caesar-lcm) or [Python](http://github.com/dehann/Caesar.jl/blob/master/examples/database/python/neo4j_interact_example.py), as listed in features below. Multi-modal (quasi-multi-hypothesis) navigation and mapping solutions, using various sensor data, is a corner stone of this package. Multi-sensor fusion is made possible via vertically integrated [Multi-modal iSAM](http://frc.ri.cmu.edu/~kaess/pub/Fourie16iros.pdf).

Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the `FactorGraph` through a graph database using [CloudGraphs.jl](https://github.com/GearsAD/CloudGraphs.jl.git), as [discussed here](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf)[1]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl][rome-url] and back-end solver [IncrementalInference.jl][iif-url].

Comments, questions and issues welcome.

## Examples

Intersection of ambiguous elevation angle from planar SONAR sensor:

<a href="http://vimeo.com/198237738" target="_blank"><img src="https://raw.githubusercontent.com/dehann/Caesar.jl/master/doc/imgs/rovasfm02.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>

Bi-modal belief

<a href="http://vimeo.com/198872855" target="_blank"><img src="https://raw.githubusercontent.com/dehann/Caesar.jl/master/doc/imgs/rovyaw90.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>

Multi-session [Turtlebot](http://www.turtlebot.com/) example of the second floor in the [Stata Center](https://en.wikipedia.org/wiki/Ray_and_Maria_Stata_Center):

<img src="https://raw.githubusercontent.com/dehann/Caesar.jl/master/doc/imgs/turtlemultisession.gif" alt="Turtlebot Multi-session animation" width="480" border="0" /></a>

Multi-modal range only example:

<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/dehann/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>

Installation
------------

Requires via ```sudo apt-get install```, see [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) for more details.

    libvtk5-qt4-dev python-vtk

Then install required Julia packages  

    julia> Pkg.add("Caesar")

Note that Database related packages will not be automatically installed. Please see section below for details.

Basic usage
-----------

Here is a basic example of using visualization and multi-core factor graph solving:

```julia
addprocs(2)
using Caesar, RoME, TransformUtils, Distributions

# load scene and ROV model (might experience UDP packet loss LCM buffer not set)
vc = startdefaultvisualization()
sc1 = loadmodel(:scene01); sc1(vc)
rovt = loadmodel(:rov); rovt(vc)


initCov = 0.001*eye(6); [initCov[i,i] = 0.00001 for i in 4:6];
odoCov = 0.0001*eye(6); [odoCov[i,i] = 0.00001 for i in 4:6];
rangecov, bearingcov = 3e-4, 2e-3

# start and add to a factor graph
fg = identitypose6fg(initCov=initCov)
tf = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )
addOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf), odoCov) ) )

visualizeallposes!(vc, fg, drawlandms=false)

addLinearArrayConstraint(fg, (4.0, 0.0), :x0, :l1, rangecov=rangecov,bearingcov=bearingcov)
visualizeDensityMesh!(vc, fg, :l1)
addLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)

solveandvisualize(fg, vc, drawlandms=true, densitymeshes=[:l1;:x2])
```

Major features
--------------

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

* Visualization through [MIT Director](https://github.com/rdeits/DrakeVisualizer.jl).
```julia
visualizeallposes(fg) # from local dictionary
drawdbdirector()      # from database held factor graph
```

* [Foveation queries](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf) to quickly organize, extract and work with big data blobs, for example looking at images from multiple sessions predicted to see the same point `[-9.0,9.0]` in the map:
```julia
neoids, syms = foveateQueryToPoint(cloudGraph,["SESS21";"SESS38";"SESS45"], point=[-9.0;9.0], fovrad=0.5 )
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

And many more, please see the examples folder.

Dependency Status
-----------------

| **Major Dependencies** |     **Status**     |    **Test Coverage**    |
|:-----------------------:|:------------------:|:------------------:|
| Caesar.jl | [![Build Status][build-img]][build-url] | [![codecov.io][cov-img]][cov-url] |
| [RoME.jl][rome-url] | [![Build Status][r-build-img]][r-build-url] | [![codecov.io][r-cov-img]][r-cov-url] |
| [IncrementalInference.jl][iif-url] | [![Build Status][iif-build-img]][iif-build-url] | [![codecov.io][iif-cov-img]][iif-cov-url] |
| [KernelDensityEstimate.jl][kde-url] | [![Build Status][kde-build-img]][kde-build-url] | [![codecov.io][kde-cov-img]][kde-cov-url] |
| [TransformUtils.jl][tf-url] | [![Build Status][tf-build-img]][tf-build-url] | [![codecov.io][tf-cov-img]][tf-cov-url] |
| [DrakeVisualizer.jl][dvis-url] | [![Build Status][dvis-build-img]][dvis-build-url] | [![codecov.io][dvis-cov-img]][dvis-cov-url] |

Database interaction layer
==========================

For using the solver on a Database layer, you simply need to switch the working API. This can be done by calling the database connection function, and following the prompt:

```julia
using Caesar
backend_config, user_config = standardcloudgraphsetup()
fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)
# and then continue as normal with the fg object, to add variables and factors, draw etc.
```

If you have access to Neo4j and Mongo services you should be able to run the [four door test](https://github.com/dehann/Caesar.jl/blob/master/test/fourdoortestcloudgraph.jl).

Go to your browser at localhost:7474 and run one of the Cypher queries to either retrieve

    match (n) return n

or delete everything:

    match (n) detach delete n

You can run the multi-modal iSAM solver against the DB using the example [MM-iSAMCloudSolve.jl](https://github.com/dehann/Caesar.jl/blob/master/examples/database/MM-iSAMCloudSolve.jl):
```
$ julia -p20
julia> using Caesar
julia> slamindb() # iterations=-1
```

Database driven Visualization can be done with either MIT's [MIT Director](https://github.com/rdeits/DrakeVisualizer.jl) (prefered), or Collections Render which additionally relies on [Pybot](http://www.github.com/spillai/pybot). For visualization using Director/DrakeVisualizer.jl:
```
$ julia -e "using Caesar; drawdbdirector()"
```

And an [example service script for CollectionsRender](https://github.com/dehann/Caesar.jl/blob/master/examples/database/DBCollectionsViewerService.jl) is also available.

## Contributors

D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. Leonard


Future targets
--------------

This is a work in progress package. Please file issues here as needed to help resolve problems for everyone!

Hybrid parametric and non-parametric optimization. Incrementalized update rules and properly marginalized 'forgetting' for sliding window type operation. We defined interprocess interface for multi-language front-end development.

References
==========

    [1]  Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: "SLAMinDB: Centralized graph
         databases for mobile robotics" IEEE International Conference on Robotics and Automation (ICRA),
         Singapore, 2017.

[cov-img]: https://codecov.io/github/dehann/Caesar.jl/coverage.svg?branch=master
[cov-url]: https://codecov.io/github/dehann/Caesar.jl?branch=master
[build-img]: https://travis-ci.org/dehann/Caesar.jl.svg?branch=master
[build-url]: https://travis-ci.org/dehann/Caesar.jl

[rome-url]: http://www.github.com/dehann/RoME.jl
[r-cov-img]: https://codecov.io/github/dehann/RoME.jl/coverage.svg?branch=master
[r-cov-url]: https://codecov.io/github/dehann/RoME.jl?branch=master
[r-build-img]: https://travis-ci.org/dehann/RoME.jl.svg?branch=master
[r-build-url]: https://travis-ci.org/dehann/RoME.jl

[iif-cov-img]: https://codecov.io/github/dehann/IncrementalInference.jl/coverage.svg?branch=master
[iif-cov-url]: https://codecov.io/github/dehann/IncrementalInference.jl?branch=master
[iif-build-img]: https://travis-ci.org/dehann/IncrementalInference.jl.svg?branch=master
[iif-build-url]: https://travis-ci.org/dehann/IncrementalInference.jl
[iif-url]: http://www.github.com/dehann/IncrementalInference.jl

[kde-cov-img]: https://codecov.io/github/dehann/KernelDensityEstimate.jl/coverage.svg?branch=master
[kde-cov-url]: https://codecov.io/github/dehann/KernelDensityEstimate.jl?branch=master
[kde-build-img]: https://travis-ci.org/dehann/KernelDensityEstimate.jl.svg?branch=master
[kde-build-url]: https://travis-ci.org/dehann/KernelDensityEstimate.jl
[kde-url]: http://www.github.com/dehann/KernelDensityEstimate.jl

[tf-cov-img]: https://codecov.io/github/dehann/TransformUtils.jl/coverage.svg?branch=master
[tf-cov-url]: https://codecov.io/github/dehann/TransformUtils.jl?branch=master
[tf-build-img]: https://travis-ci.org/dehann/TransformUtils.jl.svg?branch=master
[tf-build-url]: https://travis-ci.org/dehann/TransformUtils.jl
[tf-url]: http://www.github.com/dehann/TransformUtils.jl

[dvis-cov-img]: https://codecov.io/github/rdeits/DrakeVisualizer.jl/coverage.svg?branch=master
[dvis-cov-url]: https://codecov.io/github/rdeits/DrakeVisualizer.jl?branch=master
[dvis-build-img]: https://travis-ci.org/rdeits/DrakeVisualizer.jl.svg?branch=master
[dvis-build-url]: https://travis-ci.org/rdeits/DrakeVisualizer.jl
[dvis-url]: http://www.github.com/rdeits/DrakeVisualizer.jl
