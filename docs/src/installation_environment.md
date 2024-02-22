# Install Caesar.jl

Caesar.jl is one of the packages within the [JuliaRobotics](http://www.juliarobotics.org) community, and adheres to the [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).

## New to Julia

### Installing the Julia Binary

Although [Julia](https://julialang.org/) (or [JuliaPro](https://juliacomputing.com/)) can be installed on a Linux/Mac/Windows via a package manager, we prefer a highly reproducible and self contained (local environment) install.

The easiest method is---via the terminal---[as described on the JuliaLang.org downloads page](https://julialang.org/downloads/).
!!! note
    Feel free to modify this setup as you see fit.

## VSCode IDE Environment

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

!!! note
    For [ROS.org](https://www.ros.org/) users, see at least one usage example at [the ROS Direct page](@ref ros_direct).

## Installing Julia Packages

### Vanilla Install

The philosophy around Julia packages are discussed at length in the [Julia core documentation](https://docs.julialang.org/en/stable/manual/packages/), where each Julia package relates to a git repository likely found on [Github.com](http://www.github.com).  Also see [JuliaHub.com](https://juliahub.com/ui/Packages/Caesar/BNbRm) for dashboard-style representation of the broader Julia package ecosystem.
To install a Julia package, simply start a `julia` REPL (equally the Julia REPL in VSCode) and then type:

```julia
julia> ] # activate Pkg manager
(v___) pkg> add Caesar
```

### Version Control, Branches

These are [registered packages](https://pkg.julialang.org/) maintained by [JuliaRegistries/General](http://github.com/JuliaRegistries/General).
Unregistered latest packages can also be installed with using only the `Pkg.develop` function:

```julia
# Caesar is registered on JuliaRegistries/General
julia> ]
(v___) pkg> add Caesar
(v___) pkg> add Caesar#janes-awesome-fix-branch
(v___) pkg> add Caesar@v0.16

# or alternatively your own local fork (just using old link as example)
(v___) pkg> add https://github.com/dehann/Caesar.jl
```

### Virtual Environments

!!! note
    Julia has native support for virtual environments and exact package manifests.  See [Pkg.jl Docs](https://pkgdocs.julialang.org/v1/environments/) for more info.  More details and features regarding package management, development, version control, virtual environments are available there.

## Next Steps

The sections hereafter describe [Building](@ref building_graphs), [Interacting], and [Solving](@ref solving_graphs) factor graphs.  We also recommend reviewing the various examples available in the [Examples section](@ref examples_section).  

### Possible System Dependencies

The following (Linux) system packages have been required on some systems in the past, but likely does not have to be installed system wide on newer versions of Julia:
```
# Likely dependencies
sudo apt-get install hdf5-tools imagemagick

# optional packages
sudo apt-get install graphviz xdot
```
