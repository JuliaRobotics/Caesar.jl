# [Compile Binaries](@id compile_binaries)

## Ahead Of Time Compile RoME.so

In RoME, run the `compileRoME/compileRoMESysimage.jl` script

To use RoME with the newly created sysimage, start julia with:
```
julia -O3 -J ~/.julia/dev/RoME/compileRoME/RoMESysimage.so
```

## Static, Shared Object `.so` Compilation

Packages are already compiled to static objects (`.ji` files), but can also be compiled to more common `.so` files.  See [this AOT vs JIT compiling blog post](https://juliacomputing.com/blog/2016/02/09/static-julia.html) for a deeper discussion.  Also see [this Julia Binaries Blog](https://medium.com/@sdanisch/compiling-julia-binaries-ddd6d4e0caf4).  See recent dedicated [issue tracker here](https://github.com/JuliaRobotics/RoME.jl/issues/288).

!!! note
    [More on discourse.](https://discourse.julialang.org/t/ann-packagecompiler-with-incremental-system-images/20489).  Also see new brute force sysimg work at [Fezzik.jl](https://github.com/TsurHerman/Fezzik).