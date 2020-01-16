# Welcome

Caesar.jl is one of the packages within the [JuliaRobotics](http://www.juliarobotics.org) community, and adheres to the [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).

## Local Dependencies

The following system packages are used by Caesar.jl:
```
# required packages
sudo apt-get install hdf5-tools

# optional packages
sudo apt-get install graphviz imagemagick
```

## New to Julia and want a full Development Install

### Local Installation of Julia

Although [Julia](https://julialang.org/) (or [JuliaPro](https://juliacomputing.com/)) can be installed on a Linux computer using the `apt` package manager, we are striving for a fully local installation environment which is highly reproducible on a variety of platforms.

The easiest method is---via the terminal---to [download the desired](https://julialang.org/downloads/) version of Julia as a binary, extract, setup a symbolic link, and run:

```bash
cd ~
mkdir -p julia-software
cd julia-software
wget https://julialang-s3.julialang.org/bin/linux/x64/1.2/julia-1.2.0-linux-x86_64.tar.gz
tar -xvf julia-1.2.0-linux-x86_64.tar.gz
cd /usr/bin
sudo ln -s ~/julia-software/julia-1.2.0/bin/julia julia
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

## Setup Juno IDE Environment

[Juno IDE](http://junolab.org/) allows for interactive development of Julia code by extending the [Atom text editor](https://atom.io/) with a few packages.
Download and install Atom as instructed on the website, or via command line:

```
cd ~/Downloads
wget https://atom.io/download/deb
dpkg -i atom-amd64.deb
```

After installing and running Atom, you can choose to either install `uber-juno` package [in one go](https://github.com/JunoLab/uber-juno/blob/master/setup.md) or install the three associated packages individually.
In Atom, open the command pallette by pressing `Ctrl + Shft + p` and typing `settings`.
Go to the `install` tab, search for and install either
```
uber-juno
```
or the individual packages directly:
```
ink
julia-client
julia-language
latex-completions
```

!!! note
    Some situations have required the user separately installing the `Atom.jl` Julia package via command line (if Juno does not automatically install Atom.jl for you).  Atom.jl can then be installed with Julia's package manager and `add Atom` — see below.

There are a variety of useful packages in Atom, such as `minimap` and `minimap-git`.

To install the Julia packages related to [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl)---which are independent of the Atom packages installed above---please follow instructions below.

## Julia Packages

The philosophy around Julia packages are discussed at length in the [Julia core documentation](https://docs.julialang.org/en/stable/manual/packages/), where each Julia package relates to a git repository likely found on [Github.com](http://www.github.com).
To install a Julia package, simply open a `julia` REPL (equally the julia REPL in Atom/Juno) and type:

```julia
] # activate Pkg manager
(v1.0) pkg> add Caesar
```

These are [registered packages](https://pkg.julialang.org/) maintained by [JuliaLang/METADATA.jl](http://www.github.com/JuliaLang/METADATA.jl).
Unregistered latest packages can also be installed with using only the `Pkg.develop` function:

```julia
# Just using Caesar URL as an example --  Caesar is already registered with METADATA
using Pkg
Pkg.develop(PackageSpec(url="https://github.com/JuliaRobotics/Caesar.jl.git"))
```

Unless you change the default environment variable `JULIA_PKG_DIR`, all packages (git repos) are cloned/installed to `~/.julia`.
You can work with the packages as regular git repositories there.


### Install Visualization Tools

RoMEPlotting.jl (2D) and Arena.jl (3D) as optional visualization packages:
```julia
(v1.0) pkg> add RoMEPlotting

# separately
(v1.0) pkg> add Arena#master
```

!!! note
    Arena.jl and Amphitheater.jl are currently being refactored as part of the broader DistributedFactorGraph migration, the features are are in beta stage (1Q2020).

## Install Visualization Utils (e.g. Arena.jl)

3D Visualizations are provided by [Arena.jl](https://github.com/JuliaRobotics/Arena.jl) as well as development package Amphitheater.jl.
Please follow instructions on the [Visualizations page](concepts/arena_visualizations.md) for a variety of 3D utilities.

Arena.jl can be installed with the following steps:
```julia
]
add Arena
```

of the latest development version:
```julia
(v1.0) pkg> add Arena#master
```

## RoMEPlotting.jl for 2D plots

Previous versions of libraries required the following Linux system packages be installed:
```bash
sudo apt-get install libfontconfig1
sudo apt-get install gettext
sudo apt-get install libcairo2
sudo apt-get install libpango1.0-0  # or libpango1.0-1
```

!!! note
    As of 1Q2020 it is likely that most systems won’t require a system install of libpango or even libcairo.

Please install the latest RoMEPlotting using Package manager as follows:
```
$ julia
julia> ] # to get package manager
(v1.2) pkg> add RoMEPlotting
```

Alternatively, the `dev` command --- i.e. `(v1.2) pkg> dev RoMEPlotting` --- will clone the RoMEPlotting.jl git repository to your local `.julia/dev/RoMEPlotting` folder.

## The "I Know Julia" Installation (TL;DR)

### Install Inference Tools

Add Caesar to your Julia packages, you can install the metadata registered package 'Caesar' in Julia 1.0 with:
```julia
julia> ] # to enable package manager
(v1.0) pkg> add Caesar
```

Unit tests can further be performed for the upstream packages as follows -- **NOTE** first time runs are slow since each new function call or package must first be precompiled.
```julia
# the multimodal incremental smoothing and mapping solver
(v1.2) pkg> test IncrementalInference
...
# robotics related variables and factors to work with IncrementalInference -- can be used standalone SLAM system
(v1.2) pkg> test RoME
...
# umbrella framework with interaction tools and more -- allows stand alone and server based solving
(v1.2) pkg> test Caesar
...
```

## Install "Just the ZMQ/ROS Runtime Solver" (Linux)

Work in progress (see issue [#278](https://github.com/JuliaRobotics/Caesar.jl/issues/278)).

## Contributing, Issues, or Comments

Please feel free to open [issues with Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl/issues) or even Fork and Pull Request as required.
General conversations or comments can be made in the [Caesar Gist](https://gist.github.com/dehann/537f8a2eb9cc24d8bbd35ae92cb4d2d2).
