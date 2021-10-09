# Compile Binaries

## Ahead Of Time Compile RoME.so

In RoME, run the `compileRoME/compileRoMESysimage.jl` script

To use RoME with the newly created sysimage, start julia with:
```
julia -O3 -J ~/.julia/dev/RoME/compileRoME/RoMESysimage.so
```
