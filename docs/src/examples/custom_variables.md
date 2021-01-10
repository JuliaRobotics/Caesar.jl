
## Creating New Variables

A handy macro can help define new variables which shows a Pose2 example with 3 degrees of freedom: `X, Y, theta`:
```@docs
@defVariable
```

The macro generates the necessary code for IncrementalInference to be able to use a new user defined variable -- e.g.:
```julia
addVariable!(fg, :x0, MyVariable)
```

!!! note
    See [RoME.jl#244](http://www.github.com/JuliaRobotics/RoME.jl/issues/244) regarding plans to fundamentally integrate with [Manifolds.jl](http://www.github.com/JuliaManifolds/Manifolds.jl)

The format for defining manifolds is likely to change in the near future (2021Q1), where manual descriptions like `(:Euclid, :Euclid, :Circular)` will be replaced with a more formal definition like `SE2` or `Manifolds.ProductRepr(Euclid{2}, Rotation)`.
