# Caesar.jl

[![Build Status](https://travis-ci.org/dehann/Caesar.jl.svg?branch=master)](https://travis-ci.org/dehann/Caesar.jl)
[![codecov.io](https://codecov.io/github/dehann/Caesar.jl/coverage.svg?branch=master)](https://codecov.io/github/dehann/Caesar.jl?branch=master)

[![Caesar](http://pkg.julialang.org/badges/Caesar_0.5.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.5)
[![Caesar](http://pkg.julialang.org/badges/Caesar_0.6.svg)](http://pkg.julialang.org/?pkg=Caesar&ver=0.6)

A modern robotic toolkit for localization and mapping -- towards non-parametric / parametric navigation solutions.

This is a research and development driven project and intended to reduce the barrier of entry for Simultaneous Localization and Mapping (SLAM) systems. This [Julia](http://www.julialang.org/) package encompasses test cases and robot related software for multi-modal (multi-hypothesis) navigation and mapping solutions from various sensor data, made possible by [Multi-modal iSAM](http://frc.ri.cmu.edu/~kaess/pub/Fourie16iros.pdf). Please see related packages, Robot Motion Estimate [RoME.jl](https://github.com/dehann/RoME.jl) and back-end solver [IncrementalInference.jl](https://github.com/dehann/IncrementalInference.jl).

## Examples

Multi-modal range only example:

<a href="https://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/dehann/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="480" border="10" /></a>

Ambiguous elevation angle from planar sensor:

![alt tag](https://raw.githubusercontent.com/dehann/Caesar.jl/master/doc/imgs/directorexample01.png)

Installation
------------

    julia> Pkg.add("Caesar")

Requires via sudo apt-get, see [DrakeVisualizer.jl](https://github.com/rdeits/DrakeVisualizer.jl) for more details.

    libvtk5-qt4-dev python-vtk

Major features
--------------

* Visualization through [MIT Director](https://github.com/rdeits/DrakeVisualizer.jl).

* A multicore SLAM server over tcp

    julia -p10 -e "using Caesar; tcpStringSLAMServer()"

* A multicore Bayes 2D feature tracking server over tcp

    julia -p10 -e "using Caesar; tcpStringBRTrackingServer()"

Database interaction layer
--------------------------

For using the solver on a DataBase layer (work in progress on centralized architecture ) see [CloudGraphs](https://github.com/GearsAD/CloudGraphs.jl.git),

Install [Neo4j](https://neo4j.com/) and add these packages to your Julia system

    Pkg.clone("https://github.com/GearsAD/Neo4j.jl.git")
    Pkg.clone("https://github.com/GearsAD/CloudGraphs.jl.git")

And uncomment CloudGraphs related lines from Caesar/REQUIRE and src/Caesar.jl and test/runtests.jl Ln 7 to true.

You should be able to rerun the four door test on both internal dictionaries and repeated on Neo4j DB

    Pkg.test("Caesar")

Go to your browser at localhost:7474 and run the Cypher query

    match (n) return n

to see current graph. You can delete the graph using the query

    match (n) detach delete n


Future targets
--------------

This is a work in progress package. Please file issues here as needed to help resolve problems for everyone!

Hybrid parametric and non-parametric optimization. Incrementalized update rules and properly marginalized 'forgetting' for sliding window type operation. We defined interprocess interface for multi-language front-end development.
