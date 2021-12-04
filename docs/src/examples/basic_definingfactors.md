# [Custom Prior Factor](@id custom_prior_factor)

Julia's type inference allows overloading of member functions outside a module.  Therefore new factors can be defined at any time.  


| Required                                  | Brief description                                                                      |
|:------------------------------------------|:-------------------------------------------------------------------------------------- |
| `MyFactor`  struct                        | Prior (`<:AbstractPrior`) or Relative (`<:AbstractManifoldMinimize`) factor definition |
| `getManifold`                             | The manifold of the factor |
| `(cfo::CalcFactor{<:MyFactor})`           | Factor residual function |
| **Optional methods**                      | **Brief description**                                                                  |
| `getSample(cfo::CalcFactor{<:MyFactor})`  | Get a sample from the factor |


To better illustrate, in this example we will add new factors into the `Main` context **after** construction of the factor graph has already begun.

!!! tip
    `IIF` is a convenient `const` alias of the module `IncrementalInference`, similarly `AMP` for `ApproxManifoldProducts`.

## Defining a New Prior (`<:AbsoluteFactor`)

Now lets define our own prior, `MyPrior` which allows for arbitrary distributions that inherit from `<: IIF.SamplableBelief`:

```julia
struct MyPrior{T <: SamplableBelief} <: IIF.AbstractPrior
  Z::T
end
```

New priors must inheret from `IIF.AbstractPrior`, and usually takes a user input `<:SamplableBelief` as probabilistic model.  `<:AbstractPrior` is a unary factor that introduces absolute information about only one variable.

## [Specialized `getSample` (if `.Z`)](@id specialized_getSample)

Caesar.jl uses a convention (non-binding) to simplify factor definitions in easier cases, but not restrict more complicated cases -- a default `getSample` function already exists in `IIF` which assumes the field `.Z <: SamplableBelief` is used to generate the random sample values.  So, the example above actually does not require the user to provide a specific `getSample(cf::CalcFactor{<:MyPrior})` dispatch.  

For the sake of the tutorial, let's write one anyway.  Remember that we are now overriding the `IIF` API with a new dispatch, for that we need to `import` the function
```julia
import IncrementalInference: getSample

# adding our own specialized dispatch on getSample
IIF.getSample(cfo::CalcFactor{<:MyPrior}) = rand(cfo.factor.Z)
```

It is **important to note** that for `<:AbstractPrior` the `getSample` must return a *point* on the manifold, not a tangent vector or coordinate.  

To recap, the `getSample` function for priors returns a measurement sample as points on the manifold.

### Ready to Use

This new prior can now readily be added to an ongoing factor graph:
```julia
# lets generate a random nonparametric belief

pts = [samplePoint(getManifold(ContinuousEuclid{1}), Normal(8.0,2.0)) for _=1:75]
someBelief = manikde!(pts, ContinuousEuclid{1})

# and build your new factor as an object
myprior = MyPrior(someBelief)
```

and add it to the existing factor graph from earlier, lets say:
```julia
addFactor!(fg, [:x1], myprior)
```

Thats it, this factor is now part of the graph.  This should be a solvable graph:
```julia
solveGraph!(fg); # exact alias of solveTree!(fg)
```

Later we will see how to ensure these new factors can be properly serialized to work with features like `saveDFG` and `loadDFG`.  See [What is `CalcFactor`](@ref whatiscalcfactor) for more details.

See the next page on how to build your own [Custom Relative Factor](@ref custom_relative_factor).  Serialization of factors is also discussed in more detail at [Standardized Factor Serialization](@ref factor_serialization).