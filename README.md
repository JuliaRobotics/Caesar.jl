# Caesar.jl

A modern robotic toolkit for localization and mapping.

Towards non-parametric / parametric navigation solutions
--------------------------------------------------------

This is a research and development driven project and intended to reduce the barrier of entry for Simultaneous Localization and Mapping (SLAM) systems. This Julia package encompasses test cases and robot related software for multimodal (multihypothesis) navigation and mapping solutions from various sensor data.

[Video example](https://vimeo.com/190052649)

Installation (unregistered package)
-----------------------------------

    julia
    Pkg.clone("https://github.com/dehann/Caesar.jl.git")

Requires

    RoME, IncrementalInference # unregistered packages
    KernelDensityEstimate, Graphs, GraphViz, Gadfly, NLsolve, Distributions, DataFrames, JSON # registered packages

Major features
--------------

A multicore SLAM server over tcp

    julia -p10 -e "using Caesar; tcpStringSLAMServer()"

A multicore Bayes 2D feature tracking server over tcp

    julia -p10 -e "using Caesar; tcpStringBRTrackingServer()"

Future targets
--------------

Hybrid parametric and non-parametric optimization. Incrementalized update rules and properly marginalized 'forgetting' for sliding window type operation. We defined interprocess interface for multi-language front-end development.
