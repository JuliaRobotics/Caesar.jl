# Welcome

Caesar.jl is one of the packages within the [JuliaRobotics](http://www.juliarobotics.org) community, and adheres to the [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).

## Local Dependencies

The following system packages are used by Caesar.jl:
```
# required packages
sudo apt-get install hdf5-tools imagemagick

# optional packages
sudo apt-get install graphviz xdot
```

## New to Julia and want a full Development Install

### Local Installation of Julia

Although [Julia](https://julialang.org/) (or [JuliaPro](https://juliacomputing.com/)) can be installed on a Linux computer using the `apt` package manager, we are striving for a fully local installation environment which is highly reproducible on a variety of platforms.

The easiest method is---via the terminal---to [download the desired](https://julialang.org/downloads/) version of Julia as a binary, extract, setup a symbolic link, and run:

```bash
cd ~
mkdir -p .julia
cd .julia
wget https://julialang-s3.julialang.org/bin/linux/x64/1.5/julia-1.5.2-linux-x86_64.tar.gz
tar -xvf julia-1.5.2-linux-x86_64.tar.gz
rm julia-1.5.2-linux-x86_64.tar.gz
cd /usr/local/bin
sudo ln -s ~/.julia/julia-1.5.2/bin/julia julia
```
!!! note
    Feel free to modify this setup as you see fit.

This should allow any terminal or process on the computer to run the Julia REPL by type `julia` and testing with:

#### [Optional] Quick Test that Julia is Working 

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

## Setup VSCode IDE Environment

[VSCode IDE](https://www.julia-vscode.org/) allows for interactive development of Julia code using the Julia Extension.  After installing and running VSCode, install the Julia Language Support Extension:

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/97769200-444fdf80-1aff-11eb-8ca4-dc6d7a3830fd.png" width="800" border="0" />
</p>
```

In VSCode, open the command pallette by pressing `Ctrl + Shift + p`.  There are a wealth of tips and tricks on how to use VSCode.  See [this JuliaCon presentation for as a general introduction into 'piece-by-piece' code execution and much much more](https://www.youtube.com/watch?v=IdhnP00Y1Ks).  Working in one of the Julia IDEs like VS Code or Juno should feel something like this (Gif borrowed from [DiffEqFlux.jl](https://github.com/SciML/DiffEqFlux.jl)):
```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/1814174/88589293-e8207f80-d026-11ea-86e2-8a3feb8252ca.gif" width="800" border="0" />
</p>
```

There are a variety of useful packages in VSCode, such as `GitLens`, `LiveShare`, and `Todo Browser` as just a few highlights.  These *VSCode Extensions* are independent of the already vast JuliaLang Package Ecosystem (see [JuliaObserver.com](https://juliaobserver.com/)).

## Julia Packages

The philosophy around Julia packages are discussed at length in the [Julia core documentation](https://docs.julialang.org/en/stable/manual/packages/), where each Julia package relates to a git repository likely found on [Github.com](http://www.github.com).
To install a Julia package, simply open a `julia` REPL (equally the julia REPL in Atom/Juno) and type:

```julia
] # activate Pkg manager
(v1.5) pkg> add Caesar
```

These are [registered packages](https://pkg.julialang.org/) maintained by [JuliaRegistries/General](http://github.com/JuliaRegistries/General).
Unregistered latest packages can also be installed with using only the `Pkg.develop` function:

```julia
# Caesar is registered on JuliaRegistries/General
julia> ]
(v1.5) pkg> add Caesar
```

See [Pkg.jl](https://github.com/JuliaLang/Pkg.jl) for details and features regarding package management, development, version control, virtual environments and much more.

## Install Visualization Tools

### RoMEPlotting.jl for 2D plots

RoMEPlotting.jl (2D) and Arena.jl (3D) as optional visualization packages:
```julia
(v1.5) pkg> add RoMEPlotting
```

!!! note
    As of 1Q2020 it is likely that most systems won’t require a system install of libpango or even libcairo.
    Previous versions of libraries required the following Linux system packages be installed:
    ```bash
    sudo apt-get install libfontconfig1
    sudo apt-get install gettext
    sudo apt-get install libcairo2
    sudo apt-get install libpango1.0-0  # or libpango1.0-1
    ```

### Install 3D Visualization Utils (e.g. Arena.jl)

3D Visualizations are provided by [Arena.jl](https://github.com/JuliaRobotics/Arena.jl) as well as development package Amphitheater.jl.
Please follow instructions on the [Visualizations page](concepts/arena_visualizations.md) for a variety of 3D utilities.

!!! note
    Arena.jl and Amphitheater.jl are currently being refactored as part of the broader DistributedFactorGraph migration, the features are are in beta stage (1Q2020).

Install the latest `master` branch version with
```julia
(v1.5) pkg> add Arena#master
```

## Running Unit Tests Locally

Unit tests can further be performed for the upstream packages as follows -- **NOTE** first time runs are slow since each new function call or package must first be precompiled.
```julia
# the multimodal incremental smoothing and mapping solver
(v1.5) pkg> test IncrementalInference
...
# robotics related variables and factors to work with IncrementalInference -- can be used standalone SLAM system
(v1.5) pkg> test RoME
...
# umbrella framework with interaction tools and more -- allows stand alone and server based solving
(v1.5) pkg> test Caesar
...
```

## Install Repos for Development

Alternatively, the `dev` command:
```julia
(v1.5) pkg> dev https://github.com/JuliaRobotics/Caesar.jl
```

!!! warn
    Development packages are NOT managed by Pkg.jl, so you have to manage this Git repo manually.  Development packages can usually be found at, e.g. `Caesar`
    ```
    ~/.julia/dev/Caesar
    ```

If you'd like to modify or contribute then feel free to fork the specific repo from JuliaRobotics, complete the work on branches in the fork as is normal with a Git workflow and then submit a PR back upstream.  We try to keep PRs small, specific to a task and preempt large changes by first merging smaller non-breaking changes and finally do a small switch over PR.  We also follow a backport onto `release/vX.X` branch strategy with common `master` as the development lobby that builds successfully 99.999% of the time.

## Ahead Of Time Compile RoME.so

In RoME, run the `compileRoME/compileRoMESysimage.jl` script

To use RoME with the newly created sysimage, start julia with:
```
julia -O3 -J ~/.julia/dev/RoME/compileRoME/RoMESysimage.so
```

## Install "Just the ZMQ/ROS Runtime Solver" (Linux)

Work in progress (see issue [#278](https://github.com/JuliaRobotics/Caesar.jl/issues/278)).

## Contributing, Issues, or Comments

Please feel free to open [issues with Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl/issues) or even Fork and Pull Request as required.
General conversations or comments can be made in the [Caesar Gist](https://gist.github.com/dehann/537f8a2eb9cc24d8bbd35ae92cb4d2d2).
