# [Compile Binaries](@id compile_binaries)

Broader Julia ecosystem work on compiling shared libraries and images is hosted by [PackageCompiler.jl](https://github.com/JuliaLang/PackageCompiler.jl), see documentation there.

## Compiling RoME.so

A default RoME system image script can be used [`compileRoME/compileRoMESysimage.jl`](https://github.com/JuliaRobotics/RoME.jl/blob/master/compileRoME/compileRoMESysimage.jl) to reduce the "time-to-first-plot".

To use RoME with the newly created sysimage, start julia with:
```
julia -O3 -J ~/.julia/dev/RoME/compileRoME/RoMESysimage.so
```

Which should dramatically cut down on the load time of the included package JIT compilation.  More packages or functions can be added to the binary, depending on the application.  Furthermore, full executable binaries can easily be made with PackageCompiler.jl.

## More Info

!!! note
    Also see [this Julia Binaries Blog](https://medium.com/@sdanisch/compiling-julia-binaries-ddd6d4e0caf4).  [More on discourse.](https://discourse.julialang.org/t/ann-packagecompiler-with-incremental-system-images/20489).  Also see new brute force sysimg work at [Fezzik.jl](https://github.com/TsurHerman/Fezzik).
    
!!! note
    Contents of a previous blog post [this AOT vs JIT compiling blog post](https://juliacomputing.com/blog/2016/02/09/static-julia.html) has been wrapped into PackageCompiler.jl.
