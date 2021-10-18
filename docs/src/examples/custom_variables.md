
## Creating New Variables

A handy macro can help define new variables which shows a Pose2 example with 3 degrees of freedom: ``X, Y, \theta``.  Note that we use [Manifolds.jl as fundamental abstraction](https://juliamanifolds.github.io/Manifolds.jl/latest/examples/manifold.html) for numerical operations.  Users can choose how to represent data, for example [`RoME.Pose2`](@ref) is defined as a `Manifolds.SpecialEuclidean(2)`, and the default data representation is (but doesn't have to be) `Manifolds.identity_element(SpecialEuclidean(2))` -- i.e. likely an `Manifolds.ArrayPartition` or `Manifolds.ProductRepr` (older).
```@docs
@defVariable
```

The macro generates the necessary code for IncrementalInference to be able to use a new user defined variable -- e.g.:
```julia
addVariable!(fg, :x0, MyVariable)
```

!!! note
    Since [RoME.jl#244](http://www.github.com/JuliaRobotics/RoME.jl/issues/244) the Caesar.jl system of packages fundamentally integrated with [Manifolds.jl](http://www.github.com/JuliaManifolds/Manifolds.jl).


