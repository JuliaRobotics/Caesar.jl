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

### Optional Package Loading

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

## Running Unit Tests Locally

Unit tests can further be performed for the upstream packages as follows -- **NOTE** first time runs are slow since each new function call or package must first be precompiled.  These test can take up to an hour and may have occasional stochastic failures in any one of the many tests being run.  Thus far we have accepted occasional stochasticly driven numerical events---e.g. a test event might result in `1.03 < 1`---rather than making tests so loose such that actual bugs are missed.  Strictly speaking, we should repeat tests 10 times over with tighter tolerances, but that would require hundreds or thousands of cloud CI hours a week.
```julia
juila> ] # activate Pkg manager

# the multimodal incremental smoothing and mapping solver
(v1.6) pkg> test IncrementalInference
...
# robotics related variables and factors to work with IncrementalInference -- can be used standalone SLAM system
(v1.6) pkg> test RoME
...
# umbrella framework with interaction tools and more -- allows stand alone and server based solving
(v1.6) pkg> test Caesar
...
```

## Install Repos for Development

Alternatively, the `dev` command:
```julia
(v1.6) pkg> dev https://github.com/JuliaRobotics/Caesar.jl
```

!!! warn
    Development packages are NOT managed by Pkg.jl, so you have to manage this Git repo manually.  Development packages can usually be found at, e.g. `Caesar`
    ```
    ~/.julia/dev/Caesar
    ```

If you'd like to modify or contribute then feel free to fork the specific repo from JuliaRobotics, complete the work on branches in the fork as is normal with a Git workflow and then submit a PR back upstream.  We try to keep PRs small, specific to a task and preempt large changes by first merging smaller non-breaking changes and finally do a small switch over PR.  We also follow a backport onto `release/vX.Y` branch strategy with common `main || master` branch as the "lobby" for shared development into which individual single responsibility PRs are merged.  Each PR, the `main` development lobby, and stable `release/vX.Y` branches are regularly tested through Continuous Integration at each of the repsective packages.

!!! note
    Binary compilation and fast "first-time-to-plot" can be done through [PackageCompiler.jl, see here for more details](concepts/compile_binary.md).

## Julia Command Examples

Run Julia in REPL (console) mode:
```julia
$ julia
julia> println("hello world")
"hello world"
```

Maybe a script, or command:

```
user@...$ echo "println(\"hello again\")" > myscript.jl
user@...$ julia myscript.jl
hello again
user@...$ rm myscript.jl

user@...$ julia -e "println(\"one more time.\")"
one more time.
user@...$ julia -e "println(\"...testing...\")"
...testing...
```

!!! note
    When searching for Julia related help online, use the phrase 'julialang' instead of just 'julia'.  For example, search for 'julialang workflow tips' or 'julialang performance tips'.
    Also, see [FAQ - Why are first runs slow?](https://www.juliarobotics.org/Caesar.jl/latest/faq/#Just-In-Time-Compiling-(i.e.-why-are-first-runs-slow?)-1), which is due to Just-In-Time/Pre compiling and caching.
## Next Steps

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).  Caesar.jl also supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md).
