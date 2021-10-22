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

## Next Steps

The next section describes the initial steps in constructing and solving graphs will be discussed in the upcoming documentation page [Building and Solving Graphs](building_graphs.md).  We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).  The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](../examples/adding_variables_factors.md).  Caesar supports both in-memory solving (fast, for moderately-sized graphs) as well as [shared data persistence and inference](database_interactions.md) for massive graphs, multiple sessions, and multiple agents.

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).  Caesar.jl also supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md).
