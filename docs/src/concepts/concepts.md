# Caesar Concepts

A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion [here (author disclosure)](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1):

![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png).

# Why/Where does non-Gaussian data come from?

Gaussian error models in measurement or data cues will only be Gaussian (normally distributed) if all physics/decisions/systematic-errors/calibration/etc. has a correct algebraic model in every single circumstance.  Caesar.jl and mm-iSAM is heavily focussed on state-estimation from a plethora of heterogenous data.  Four major categories of non-Gaussian errors have thus far been considered:
- Uncertain decisions (a.k.a. data association), such as a robot trying to decide if a navigation loop-closure can be deduced from a repeat observation of a similar object or measurement from current and past data.  These issues are commonly also referred to as multi-hypothesis.
- Under-determined or under-defined systems where there are more variables than constraining measurements to fully define the system as a single mode---a.k.a solution ambiguity.  For example, in 2D consider two range measurements resulting in two possible locations through trilateration.
- Nonlinearity.  For example in 2D, consider a Pose2 odometry where the orientation is uncertain:  The resulting belief of where a next pose might be (convolution with odometry factor) results in a banana shape curve, even though the entire process is driven by assumed Gaussian belief.
- Physics of the measurement process.  Many, if not all measurement processes exhibit non-Gaussian behaviour.  For example, acoustic/radio time-of-flight measurements, using either pulse-train or matched filtering, result in an "energy intensity" over time/distance of what the range to a scattering-target/source might be--i.e. highly non-Gaussian.

# Getting Started with Caesar

This section discusses the various concepts in the Caesar framework.

## Loading Packages with Multicore

!!! note
    Keywords: parallel processing, multi-threading, multi-process

The Julia is a high-performance, parallel processing enable programming language from the ground up.  Caesar.jl utilizes features from native Julia which supports at least four styles of multiprocessing: i) separate memory multi-process; ii) shared memory multi-threading; iii) asynchronous shared-memory (forced-atomic) co-routines; and iv) multi-architecture such as JuliaGPU.  As of Julia 1.4, the most reliable method of loading all code into all contexts (for multi-processor speedup) is as follows.

Make sure the environment variable `JULIA_NUM_THREADS` is set as default or per call, anywhere between 1 and 50 and recommended to use 4 as starting point.
```julia
JULIA_NUM_THREADS=6 julia -O3
   _       _ _(_)_     |  Documentation: https://docs.julialang.org
  (_)     | (_) (_)    |
   _ _   _| |_  __ _   |  Type "?" for help, "]?" for Pkg help.
  | | | | | | |/ _` |  |
  | | |_| | | | (_| |  |  Version 1.5.0 (2020-08-01)
 _/ |\__'_|_|_|\__'_|  |  Official https://julialang.org/ release
|__/                   |

julia>

# load the required packages into procid()==1
using Flux, RoME, Caesar, RoMEPlotting

# then start more processes
using Distributed
addprocs(8) # note this yields 6*8=40 possible processing threads

# now make sure all code is loaded everywhere (for separate memory cases)
@everywhere using Flux, RoME, Caesar
```

It might also be convenient to warm up some of the Just-In-Time compiling:
```julia
# solve a few graphs etc, to get majority of solve code compiled before running a robot.
[warmUpSolverJIT() for i in 1:3];
```

The best way to avoid compile time (when not developing) is to use the established Julia "first time to plot" approach based on PackageCompiler.jl, and more details are provided at [Ahead of Time compiling](https://juliarobotics.org/Caesar.jl/latest/installation_environment/#Ahead-Of-Time-Compile-RoME.so), and a few common questions might be answered via [FAQ here](https://juliarobotics.org/Caesar.jl/latest/faq/#Static,-Shared-Object-.so-Compilation).

## Building Factor Graphs

The initial steps in constructing and solving graphs will be discussed in the upcoming documentation page [Building and Solving Graphs](building_graphs.md).

## A Few Examples

We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).

## Extending Caesar
The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](adding_variables_factors.md).

## Connectivity and Extensibility
Caesar supports both in-memory solving (really fast, but for moderately-sized graphs) as well as database-driven solving (think massive graphs and multiple sessions). This is still under development/being refactored, and is discussed in [Common Data Persistence and Inference](database_interactions.md).

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).

# Visualization

Caesar supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md)
