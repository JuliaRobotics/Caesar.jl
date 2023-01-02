# Parallel Processing

!!! note
    Keywords: parallel processing, multi-threading, multi-process

Julia allows [high-performance, parallel processing from the ground up](https://docs.julialang.org/en/v1/manual/parallel-computing/).  Depending on the configuration, Caesar.jl can utilize a combination of four styles of multiprocessing: i) separate memory multi-process; ii) shared memory multi-threading; iii) asynchronous shared-memory (forced-atomic) co-routines; and iv) multi-architecture such as JuliaGPU.  As of Julia 1.4, the most reliable method of loading all code into all contexts (for multi-processor speedup) is as follows.

## Multiprocessing

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

## Start-up Time

The best way to avoid compile time (when not developing) is to use the established Julia "first time to plot" approach based on PackageCompiler.jl, and more details are provided at [Ahead of Time compiling](@ref compile_binaries).

## Multithreading

Julia has strong support for shared-memory multithreading.  The most sensible breakdown into threaded work is either within each factor calculation or across individual samples of a factor calculation.  Either of these cases require some special considerations.

### Threading Within the Residual

A factor residual function itself can be broken down further into threaded operations.  For example, see many of the features available at [JuliaSIMD/LoopVectorization.jl](https://github.com/JuliaSIMD/LoopVectorization.jl).  It is recommended to keep memory allocations down to zero, since the solver code will call on the factor samping and residual funtions mulitple times in random access.  Also keep in mind the interaction between conventional thread pool balancing and the newer [PARTR cache senstive automated thread scheduling](https://julialang.org/blog/2019/07/multithreading/).

### Threading Across Parallel Samples [DEPRECATED -- REFACTORING]

IncrementalInference.jl internally has the capability to span threads across samples in parallel computations during convolution operations.  Keep in mind which parts of residual factor computation is shared memory.  Likely the best course of action is for the factor definition to pre-allocate `Threads.nthreads()` many memory blocks for factor in-place operations.

To use this feature, IIF must be told that there are no data race concerns with a factor.  The current API uses a keyword argument on [`addFactor!`](@ref):
```julia
# NOTE, legacy `threadmodel=MultiThreaded` is being refactored with new `CalcFactor` pattern
addFactor!(fg, [:x0; :x1], MyFactor(...))
```

!!! warning
    The current IIF factor multithreading interface is likely to be reworked/improved in the near future (penciled in for 1H2022).

See page [Custom Factors](@ref custom_relative_factor) for details on how factor computations are represented in code.  Regarding threading, consider for example `OtherFactor.userdata`.  The residual calculations from different threads might create a data race on `userdata` for some volatile internal computation.  In that case it is recommended the to instead use `Threads.nthreads()` and `Threads.threadid()` to make sure the shared-memory issues are avoided:
```julia
struct MyThreadSafeFactor{T <: SamplableBelief} <: IIF.AbstractManifoldMinimize
  Z::T
  inplace::Vector{MyInplaceMem}
end

# helper function
MyThreadSafeFactor(z) = MyThreadSafeFactor(z, [MyInplaceMem(0) for i in 1:Threads.nthreads()])

# in residual function just use `thr_inplace = cfo.factor.inplace[Threads.threadid()]`
```

!!! note
    Beyond the cases discussed above, other features in the IncrementalInference.jl code base (especially regarding the Bayes tree) are already multithreaded.


## Factor Caching (In-place operations)

In-place memory operations for factors can have a significant performance improvement.  See the [Cache and Stash section](@ref section_stash_and_cache) for more details.