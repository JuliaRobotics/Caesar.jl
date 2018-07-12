# Getting Started

Caesar.jl is one of the packages within the [JuliaRobotics](http://www.juliarobotics.org) community, and adheres to the [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).

## Local copy of Julia

Although [Julia](https://julialang.org/) (or [JuliaPro](https://juliacomputing.com/)) can be installed on a Linux computer using the `apt` package manager, we are striving for a fully local installation environment which is highly reproducible on a variety of platforms.

The easiest method is---via the terminal---to [download the desired](https://julialang.org/downloads/) version of Julia as a binary, extract, setup a symbolic link, and run:

```bash
cd ~
mkdir -p julia-software
cd julia-software
wget https://julialang-s3.julialang.org/bin/linux/x64/0.6/julia-0.6.4-linux-x86_64.tar.gz
tar -xvf julia-0.6.4-linux-x86_64
cd /usr/bin
sudo ln -s ~/julia-software/julia-9d11f62bcb/bin/julia julia064
sudo ln -s julia064 julia
```
**NOTE** Feel free to modify this setup as you see fit.

This should allow any terminal or process on the computer to run the Julia REPL:

```
user@...$ julia

   _       _ _(_)_     |  A fresh approach to technical computing
  (_)     | (_) (_)    |  Documentation: https://docs.julialang.org
   _ _   _| |_  __ _   |  Type "?help" for help.
  | | | | | | |/ _` |  |
  | | |_| | | | (_| |  |  Version 0.6.3 (2018-05-28 20:20 UTC)
 _/ |\__'_|_|_|\__'_|  |  Official http://julialang.org/ release
|__/                   |  x86_64-pc-linux-gnu

julia> println("hello world")
hello world
```

Maybe a script, or command:

```
user@...$ echo "println(\"hello again\")" > myscript.jl
user@...$ julia myscript.jl
hello again
user@...$ rm myscript.jl

user@...$ julia -e "println(\"one more time.\")"
one more time.
```

**NOTE** When searching for Julia related help online, use the phrase 'julialang' instead of just 'julia'.
For example, search for 'julialang workflow tips' or 'julialang performance tips'.

## Just-In-Time Compiling (i.e. why are first runs slow?)

Julia uses just-in-time compilation ([unless pre-compiled](https://stackoverflow.com/questions/40116045/why-is-julia-taking-a-long-time-on-the-first-call-into-my-module))
 which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use.

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
Go to the `install` tab, search for and install these packages:
```
ink
julia-client
julia-language
```

**NOTE** Some situations have required separately installing the `Atom.jl` Julia package via command line and Julia REPL; this is done with Julia's package manager and `Pkg.add("Atom")`:

```
$ julia
...
julia> Pkg.add("Atom")
```

There are a variety of useful packages in Atom, such as `minimap` and `minimap-git`.

To install the Julia packages related to [Caesar.jl]()---which are independent of the Atom packages installed above---please follow instructions below.

## Julia Packages

The philosophy around Julia packages are discussed at length in the [Julia core documentation](https://docs.julialang.org/en/stable/manual/packages/), where each Julia package relates to a git repository likely found on [Github.com](http://www.github.com).
To install a Julia package, simply open a `julia` REPL (equally the julia REPL in Atom/Juno) and type:

```julia
Pkg.add("Caesar")
```

These are [registered packages](https://pkg.julialang.org/) maintained by [JuliaLang/METADATA.jl](http://www.github.com/JuliaLang/METADATA.jl).
Unregistered packages can also be installed with using only the `Pkg.clone` function:

```julia
# Just using Caesar URL as an example --  Caesar is already registered with METADATA
Pkg.clone("https://github.com/JuliaRobotics/Caesar.jl.git")
```

Unless you change the default environment variable `JULIA_PKG_DIR`, all packages (git repos) are cloned/installed to `~/.julia`.
You can work with the packages as regular git repositories there.

## Install Visualization Utils (e.g. Arena.jl)

Visualizations were removed from Caesar by popular demand and moved to a new package [Arena.jl](https://github.com/JuliaRobotics/Arena.jl) instead.
Please follow instructions on the [Visualizations page](http://www.juliarobotics.org/Caesar.jl/latest/arena_visualizations.html) for a variety of 2D / 3D utilities.

## Contributing, Issues, or Comments

Please feel free to open [issues with Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl/issues) or even Fork and Pull Request as required.
General conversations or comments can be made in the [Caesar Gist](https://gist.github.com/dehann/537f8a2eb9cc24d8bbd35ae92cb4d2d2).
