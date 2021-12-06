# [Custom Variables](@id custom_variables)

In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications.  Let's look at creating custom variables first.

A handy macro can help define new variables:
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

Another good example to look at is RoME's [`Pose2`](@ref) with 3 degrees of freedom: ``X, Y`` and a rotation matrix using ``R(\theta)``.  Caesar.jl uses the [JuliaManifolds/Manifolds.jl](https://github.com/JuliaManifolds/Manifolds.jl) for defining numerical operations, we can use the `Manifolds.ProductRepr` (or [`RecursiveArrayTools.ArrayPartition`](https://github.com/SciML/RecursiveArrayTools.jl)), to define manifold point types:
```julia
# already exists in RoME/src/factors/Pose2D.jl
@defVariable(
    Pose2,
    SpecialEuclidean(2),
    ProductRepr(MVector{2}(0.0,0.0), MMatrix{2,2}(1.0,0.0,0.0,1.0))
)
```

Here we used `Manifolds.SpecialEuclidean(2)` as the variable manifold, and the default data representation is similar to `Manifolds.identity_element(SpecialEuclidean(2))`, or `Float32[1.0 0; 0 1]`, etc.  In the example above, we used `StaticArrays.MVector`, `StaticArrays.MMatrix` for better performance, owing to better heap vs. stack memory management.
```@docs
@defVariable
```

!!! note
    Users can implement their own manifolds using the [ManifoldsBase.jl API](https://juliamanifolds.github.io/Manifolds.jl/latest/interface.html); also see [the tutorial](https://juliamanifolds.github.io/Manifolds.jl/stable/examples/manifold.html#manifold-tutorial).  See [JuliaManifolds/Manifolds.jl](https://github.com/JuliaManifolds/Manifolds.jl) for general information.
