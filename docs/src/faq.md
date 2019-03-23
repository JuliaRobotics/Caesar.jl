## Frequently Asked Questions

### Why Julia
The implementation language of choice is Julia for a wide variety of reasons, such as agile development along side modern, high speed, type safe, multi-processor, JIT-dynamic, cross compiling (gcc and clang) and cross-platform technologies -- also see [JuliaCon2018 highlights video](https://www.youtube.com/watch?v=baR02tlea5Y).  Julia can be thought of as either {C+, Mex (done right), or a Fortran replacement}.  

### Is Caesar.jl limited to Julia? No.
The Caesar.jl project is expressly focused on making this algorithmic code available to [C/Fortran](https://docs.julialang.org/en/v1/manual/calling-c-and-fortran-code/)/[C++](https://juliacomputing.com/blog/2017/12/01/cxx-and-cxxwrap-intro.html)/C#/[Python](https://github.com/JuliaPy/PyCall.jl)/[Java](https://github.com/JuliaInterop/JavaCall.jl)/JS.  Julia itself offers [many additional interops](https://github.com/JuliaInterop).  ZMQ and HTTP/WebSockets are the standardized interfaces of choice, please see [details at the multi-language section](http://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/)).  Consider opening issues or getting in touch for more information.

### Can Julia be Embedded from C/C++
Yes, see [the Julia embedding documentation page](https://docs.julialang.org/en/v1/manual/embedding/index.html).

### Just-In-Time Compiling (i.e. why are first runs slow?)
Julia uses just-in-time compilation ([unless pre-compiled](https://stackoverflow.com/questions/40116045/why-is-julia-taking-a-long-time-on-the-first-call-into-my-module))
 which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use.
