# Caesar Concepts

A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, see a broader discussion [in realted literature here](https://juliarobotics.org/Caesar.jl/latest/refs/literature/):

![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png).

# Why/Where does non-Gaussian data come from?

Gaussian error models in measurement or data cues will only be Gaussian (normally distributed) if all physics/decisions/systematic-errors/calibration/etc. has a correct algebraic model in all circumstances.  Caesar.jl and MM-iSAMv2 is heavily focussed on state-estimation from a plethora of heterogenous data that may not yet have perfect algebraic models.  Four major categories of non-Gaussian errors have thus far been considered:
- Uncertain decisions (a.k.a. data association), such as a robot trying to decide if a navigation loop-closure can be deduced from a repeat observation of a similar object or measurement from current and past data.  These issues are commonly also referred to as multi-hypothesis.
- Underdetermined or underdefined systems where there are more variables than constraining measurements to fully define the system as a single mode---a.k.a solution ambiguity.  For example, in 2D consider two range measurements resulting in two possible locations through trilateration.
- Nonlinearity.  For example in 2D, consider a Pose2 odometry where the orientation is uncertain:  The resulting belief of where a next pose might be (convolution with odometry factor) results in a banana shape curve, even though the entire process is driven by assumed Gaussian belief.
- Physics of the measurement process.  Many measurement processes exhibit non-Gaussian behaviour.  For example, acoustic/radio time-of-flight measurements, using either pulse-train or matched filtering, result in an "energy intensity" over time/distance of what the range to a scattering-target/source might be--i.e. highly non-Gaussian.

# Getting Started with Caesar

This section discusses the various concepts in the Caesar framework.

## Julia and Help

When launching the REPL in a terminal or and IDE like VS Code (see link for documtation website):
```bash
$ julia -O3
               _
   _       _ _(_)_     |  Documentation: https://docs.julialang.org
  (_)     | (_) (_)    |
   _ _   _| |_  __ _   |  Type "?" for help, "]?" for Pkg help.
  | | | | | | |/ _` |  |
  | | |_| | | | (_| |  |  Version 1.5.2 (2020-09-23)
 _/ |\__'_|_|_|\__'_|  |  Official https://julialang.org/ release
|__/                   |
```

The `-O 3` is for level 3 code compilation optimization and is a useful habit for slightly faster execution, but slightly slower first run just-in-time compilation of any new function.

To get help with a function, just start with the `?` character followed by the function name, e.g.:
```julia
?sin
# help?> sin
search: sin sinh sind sinc sinpi sincos sincosd SingleThreaded SingularException asin using isinf asinh asind isinteger isinteractive

  sin(x)

  Compute sine of x, where x is in radians.

  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  sin(A::AbstractMatrix)

  Compute the matrix...
```

## Loading Packages

Assuming you just loaded an empty REPL, or at the start of a script, or working inside the VSCode IDE, the first thing to do is load the necessary Julia packages.  Caesar.jl is an umbrella package potentially covering over 100 Julia Packages.  For this reason the particular parts of the code are broken up amongst more focussed *vertical purpose* library packages.  Usually for Robotics either `Caesar` or less expansive `RoME` will do.  Other non-Geometric sensor processing applications might build in the MM-iSAMv2, Bayes tree, and DistributedFactorGraph libraries.  Any of these packages can be loaded as follows:

```julia
# umbrella containing most functional packages including RoME
using Caesar
# contains the IncrementalInference and other geometric manifold packages
using RoME
# contains among others DistributedFactorGraphs.jl and ApproxManifoldProducts.jl
using IncrementalInference
```

### Requires.jl for Optional Package Loading

Many of these packages have additional features that are not included by default.  For example, the [Flux.jl](https://fluxml.ai/Flux.jl/stable/) machine learning package will introduce several additional features when loaded, e.g.:
```julia
julia> using Flux, RoME

[ Info: IncrementalInference is adding Flux related functionality.
[ Info: RoME is adding Flux related functionality.
```

For completeness, so too with packages like `Images.jl`, `RobotOS.jl`, and others:
```julia
using Caesar, Images
```

## Parallel Processing

!!! note
    Keywords: parallel processing, multi-threading, multi-process

The Julia allows [high-performance, parallel processing from the ground up](https://docs.julialang.org/en/v1/manual/parallel-computing/).  Depending on configuration, Caesar.jl can utilizes any of four styles of multiprocessing: i) separate memory multi-process; ii) shared memory multi-threading; iii) asynchronous shared-memory (forced-atomic) co-routines; and iv) multi-architecture such as JuliaGPU.  As of Julia 1.4, the most reliable method of loading all code into all contexts (for multi-processor speedup) is as follows.

### Multithreading and Multiprocessing

Make sure the environment variable `JULIA_NUM_THREADS` is set as default or per call and recommended to use 4 as starting point.
```
JULIA_NUM_THREADS=4 julia -O3
```

In addition to multithreading, Caesar.jl utilizes multiprocessing to distribute computation during the inference steps.  Following standard Julia, more processes can be added as follows:
```julia
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


The next section describes the initial steps in constructing and solving graphs will be discussed in the upcoming documentation page [Building and Solving Graphs](building_graphs.md).  We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).  The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](adding_variables_factors.md).  Caesar supports both in-memory solving (fast, for moderately-sized graphs) as well as [shared data persistence and inference](database_interactions.md) for massive graphs, multiple sessions, and multiple agents.

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).  Caesar.jl also supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md).
