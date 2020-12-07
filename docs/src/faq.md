# Frequently Asked Questions

## Factor Graphs: why not just filter? 

Why can't I just filter, or what is the connection with FGs? See [the "Principles" section](https://juliarobotics.org/Caesar.jl/latest/principles/filterCorrespondence/) in the documentation. 

## Why worry about non-Gaussian Probabilities

The [non-Gaussian/multimodal section in the docs](https://juliarobotics.org/Caesar.jl/latest/concepts/concepts/#Why/Where-does-non-Gaussian-data-come-from?-1) is dedicated to precisely this question.

## Why Julia
The [JuliaLang](https://julialang.org/) and ([JuliaPro](https://juliacomputing.com/)) is an open-source Just-In-Time (JIT) & optionally precompiled, strongly-typed, and high-performance programming language.
The algorithmic code is implemented in Julia for many reasons, such as agile development, high level syntax, performance, type safety, multiple dispatch replacement for [object oriented](https://invenia.github.io/blog/2019/10/30/julialang-features-part-1/) which exhibits [several emergent properties](https://invenia.github.io/blog/2019/11/06/julialang-features-part-2/), parallel computing, dynamic development, cross compilable (with gcc and clang) and foundational cross-platform ([LLVM](http:///www.llvm.org)) technologies.  
See [JuliaCon2018 highlights video](https://www.youtube.com/watch?v=baR02tlea5Y).  Julia can be thought of as either {C+, Mex (done right), [or as a modern Fortran replacement](https://arstechnica.com/science/2020/10/the-unreasonable-effectiveness-of-the-julia-programming-language/)}.

### Current Julia version?
Caesar.jl and packages are currently targeting [Julia version](https://julialang.org/downloads/) as per the [local install page](https://juliarobotics.org/Caesar.jl/latest/installation_environment/).

### Just-In-Time Compiling (i.e. why are first runs slow?)
Julia uses just-in-time compilation ([unless already pre-compiled](https://stackoverflow.com/questions/40116045/why-is-julia-taking-a-long-time-on-the-first-call-into-my-module))
 which takes additional time the first time a new function is called. Additional calls to a cached function are fast from the second call onwards since the static binary code is now cached and ready for use.

### How does garbage collection work?

A short description of Julia's garbage collection is [described in Discourse here](https://discourse.julialang.org/t/details-about-julias-garbage-collector-reference-counting/18021/3).

!!! note
    Garbage collection can be influenced in a few ways to allow more certainty about operational outcome, see the [Julia Docs Garbage Collection Internal functions](https://docs.julialang.org/en/v1/base/base/#Internals-1) like `enable`, `preserve`, `safepoint`, etc.

### Using Julia in real-time systems?

See the JuliaCon presentation by [rdeits here](https://www.youtube.com/watch?v=dmWQtI3DFFo).

## Can Caesar.jl be used in other languages beyond Julia? Yes.
The Caesar.jl project is expressly focused on making this algorithmic code available to [C/Fortran](https://docs.julialang.org/en/v1/manual/calling-c-and-fortran-code/)/[C++](https://juliacomputing.com/blog/2017/12/01/cxx-and-cxxwrap-intro.html)/C#/[Python](https://github.com/JuliaPy/PyCall.jl)/[Java](https://github.com/JuliaInterop/JavaCall.jl)/JS.  Julia itself offers [many additional interops](https://github.com/JuliaInterop).  ZMQ and HTTP/WebSockets are the standardized interfaces of choice, please see [details at the multi-language section](https://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/)).  Consider opening issues or getting in touch for more information.

### Static, Shared Object `.so` Compilation

Packages are already compiled to static objects (`.ji` files), but can also be compiled to more common `.so` files.  See [this AOT vs JIT compiling blog post](https://juliacomputing.com/blog/2016/02/09/static-julia.html) for a deeper discussion.  Also see [this Julia Binaries Blog](https://medium.com/@sdanisch/compiling-julia-binaries-ddd6d4e0caf4).  See recent dedicated [issue tracker here](https://github.com/JuliaRobotics/RoME.jl/issues/288).  Initial work is for system image is [described in the docs here](https://juliarobotics.org/Caesar.jl/latest/installation_environment/#Ahead-Of-Time-Compile-RoME.so-1).

!!! note
    [recent developments announced on discourse.](https://discourse.julialang.org/t/ann-packagecompiler-with-incremental-system-images/20489).  Also see new brute force sysimg work at [Fezzik.jl](https://github.com/TsurHerman/Fezzik).

### Can Julia be Embedded into C/C++
Yes, see [the Julia embedding documentation page](https://docs.julialang.org/en/v1/manual/embedding/index.html).

### ROS Integration

ROS and ZMQ interfaces are closely related.  Please see the [ROS Integration Page](examples/using_ros.md) for details on using ROS with Caesar.jl.

### Why ZMQ Middleware Layer (multilang)?
[Zero Message Queue (ZMQ)](https://zeromq.org/) is a widely used data transport layer used to build various other multiprocess middleware with wide support among other programming languages.  Caesar.jl has on been used with a direct ZMQ type link, which is similar to a ROS workflow.  Contributions are welcome for binding ZMQ endpoints for a non-ROS messaging interface.

> **Note** ZMQ work has been happening on and off based on behind the main priority on resolving abstractions with the DistributedFactorGraphs.jl framework.  See ongoing work for [the ZMQ interface](https://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/#ZMQ-Messaging-Interface-1).

## What is supersolve?

When multiple numerical values/solutions exists for the (or nearly) same factor graph -- then solutions, including a reference solution (ground truth) can just be stacked in that variable.  See and comment on [a few cases here](https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/issues/182#issuecomment-545979307).

## Variable Scope in For loop Error
Julia wants you to be specific about `global` variables, and variables packed in a development script at top level are created as globals.  Globals can be accessed using the `global varname` at the start of the context.  When writing for loops (using Julia versions 0.7 through 1.3) stricter rules on global scoping applied.  The purest way to ensure scope of variables are properly managed in the REPL or Juno script Main context is using the `let` syntax (not required post Julia 1.4).
```julia
fg = ...
tree, smt, hist = solveTree!(fg)
...
# and then a loop here:
let tree=tree, fg=fg
for i 2:100
   # global tree, fg # forcing globals is the alternative
   # add variables and stuff
   ...
   # want to solve again
   tree, smt, hist = solveTree!(fg, tree)
   ...
   # more stuff
end
end # let block
```
[See Stack overflow on `let`](https://stackoverflow.com/questions/51930537/scope-of-variables-in-julia) or [the Julia docs page on scoping](https://docs.julialang.org/en/v1/manual/variables-and-scoping/index.html).  Also note it is good practice to use local scope (i.e. inside a function) variables for performance reasons.

!!! note
    This behaviour is going to change in Julia 1.5 back to what Julia 0.6 was in interactive cases, and therefore likely less of a problem in future versions.  See Julia 1.5 Change Notes, ([#28789], [#33864]).

## How to Enable `@debug` Logging.jl

https://stackoverflow.com/questions/53548681/how-to-enable-debugging-messages-in-juno-julia-editor

## Juila Image Axis Convention

Julia Images.jl follows the common ``::Array` column-major---i.e. vertical-major---index convention
  - That is `img[vertical, horizontal]`
  - See https://evizero.github.io/Augmentor.jl/images/#Vertical-Major-vs-Horizontal-Major-1 for more details.

## How does JSON-Schema work?

Caesar.jl intends to follow [json-schema.org](http://www.json-schema.org), see [step-by-step guide here](https://json-schema.org/learn/getting-started-step-by-step.html).

## How to get Julia memory allocation points?

See [discourse discussion](https://discourse.julialang.org/t/way-to-show-where-memory-allocations-occur/2161/3).

## Increase Linux Open File Limit?

If you see the error "Open Files Limit", please [follow these intructions on your local system](https://easyengine.io/tutorials/linux/increase-open-files-limit/).  This is likely to happen when debug code and a large number of files are stored in the general solution specific logpath.
