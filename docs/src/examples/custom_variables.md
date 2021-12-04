# [Custom Variables](@id custom_variables)

In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications.  Let's look at creating custom variables first.

A handy macro can help define new variables which shows a [`Pose2`](@ref) example with 3 degrees of freedom: ``X, Y, \theta``.  Caesar.jl uses [Manifolds.jl as fundamental abstraction](https://juliamanifolds.github.io/Manifolds.jl/latest/examples/manifold.html) for defining numerical operations.
```julia
@defVariable(
    MyVar,
    TranslationGroup(2),
    MVector{2}(0.0,0.0)
)
```

First we define the name `MyVar`, then the manifold on which the variable probability estimates exist (a simple Cartesian translation in two dimensions).  The third parameter is a default point for your new variable.

This new variable is now ready to be added to a factor graph:
```julia
addVariable!(fg, :myvar1, MyVar)
```

Users can choose how to represent data, for example [`RoME.Pose2`](@ref) is defined as a `Manifolds.SpecialEuclidean(2)`, and the default data representation is (but doesn't have to be) `Manifolds.identity_element(SpecialEuclidean(2))` -- i.e. likely an `Manifolds.ArrayPartition` or `Manifolds.ProductRepr` (older).
```@docs
@defVariable
```

!!! note
    Since [RoME.jl#244](http://www.github.com/JuliaRobotics/RoME.jl/issues/244) the Caesar.jl system of packages fundamentally integrated with [Manifolds.jl](http://www.github.com/JuliaManifolds/Manifolds.jl).