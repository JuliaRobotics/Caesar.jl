# Using Julia

While Caesar.jl is accessible from various programming languages, this page describes how to use Julia, existing packages, multi-process and multi-threading features, and more.  A wealth of general Julia resources are available in the Internet, see [`www.julialang.org](http://www.julialang.org) for more resources.

If you are familar with Julia, feel free to skip over to the next page.

## Julia REPL and Help

Julia's documentation on the REPL can [be found here](https://docs.julialang.org/en/v1/stdlib/REPL/).  As a brief example, the REPL in a terminal looks as follows:

```bash
$ julia -O3
               _
   _       _ _(_)_     |  Documentation: https://docs.julialang.org
  (_)     | (_) (_)    |
   _ _   _| |_  __ _   |  Type "?" for help, "]?" for Pkg help.
  | | | | | | |/ _` |  |
  | | |_| | | | (_| |  |  Version 1.6.3 (2021-09-23)
 _/ |\__'_|_|_|\__'_|  |  Official https://julialang.org/ release
|__/                   |

julia> ? # upon typing ?, the prompt changes (in place) to: help?>

help?> string
search: string String Cstring Cwstring RevString randstring bytestring SubString

  string(xs...)

  Create a string from any values using the print function.
  ...
```

The `-O 3` is for level 3 code compilation optimization and is a useful habit for slightly faster execution, but slightly slower first run just-in-time compilation of any new function.


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

Julia allows [high-performance, parallel processing from the ground up](https://docs.julialang.org/en/v1/manual/parallel-computing/).  Depending on the configuration, Caesar.jl can utilize a combination of four styles of multiprocessing: i) separate memory multi-process; ii) shared memory multi-threading; iii) asynchronous shared-memory (forced-atomic) co-routines; and iv) multi-architecture such as JuliaGPU.  As of Julia 1.4, the most reliable method of loading all code into all contexts (for multi-processor speedup) is as follows.

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

The next section describes the initial steps in constructing and solving graphs will be discussed in the upcoming documentation page [Building and Solving Graphs](building_graphs.md).  We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).  The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](../examples/adding_variables_factors.md).  Caesar supports both in-memory solving (fast, for moderately-sized graphs) as well as [shared data persistence and inference](database_interactions.md) for massive graphs, multiple sessions, and multiple agents.

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).  Caesar.jl also supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md).
