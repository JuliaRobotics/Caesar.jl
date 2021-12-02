# [Custom Factors](@id custom_factors)

Julia's type inference allows overloading of member functions outside a module.  Therefore new factors can be defined at any time.  


| Required                                  | Brief description                                                                      |
|:------------------------------------------|:-------------------------------------------------------------------------------------- |
| `MyFactor`  struct                        | Prior (`<:AbstractPrior`) or Relative (`<:AbstractManifoldMinimize`) factor definition |
| `getManifold`                             | The manifold of the factor |
| `(cfo::CalcFactor{<:MyFactor})`           | Factor residual function |
| **Optional methods**                      | **Brief description**                                                                  |
| `getSample(cfo::CalcFactor{<:MyFactor})`  | Get a sample from the factor |


To better illustrate, in this example we will add new factors into the `Main` context **after** construction of the factor graph has already begun.

```julia
using IncrementalInference

# You must import this to later overload
import IncrementalInference: getSample
```

Let's start with some factor graph:
```julia
# empty factor graph object
fg = initfg()

# add a variable in 1 dimension
addVariable!(fg, :x0, ContinuousEuclid{1})
addVariable!(fg, :x1, ContinuousEuclid{1})

# and a basic IIF.Prior from existing factors and Distributions.jl
pr0 = Prior(Normal(0,1))
addFactor!(fg, [:x0], pr0)

# making fg slightly more interesting -- i.e. x0 and x1 are 10 units apart on 1D Euclidean space
addFactor!(fg, [:x0; :x1], LinearRelative(Normal(10,1)))
```

!!! tip
    `IIF` is a convenient `const` alias of the module `IncrementalInference`, similarly `AMP` for `ApproxManifoldProducts`.

## Defining a New Prior (Absolute / Gauge)

Now lets define in `Main` scope our own prior, `MyPrior` which allows for arbitrary distributions that inherit from `<: IIF.SamplableBelief`:

```julia

struct MyPrior{T <: SamplableBelief} <: IIF.AbstractPrior
  Z::T
end

# import IncrementalInference: getSample
# sampling function
getSample(cfo::CalcFactor{<:MyPrior}) = rand(cfo.factor.Z,1)[:]
```

Note the following critical aspects that allows IIF to use the new definition:
- `<:AbstractPrior` as a unary factor that introduces absolute (or gauge) information about only one variable.
- `getSample` is overloaded with dispatch on:
  - `(cfo::CalcFactor{<:MyPrior})`
- `getSample` must return a point on the manifold for `<:AbstractPrior` factors that matches the point representation of the variable.

To recap, the new `getSample` function in this example factor returns a measurement which is of type `::Vector{Float64}}`.

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

Later we will see how to ensure these new factors can be properly serialized to work with features like `saveDFG` and `loadDFG`.

### What is `CalcFactor`

`CalcFactor` is part of the `IIF` interface to all factors.  It contains metadata and other important bits of information that are useful in a wide swath of applications.  As work requires more interesting features from the code base, it is likely that the `cfo::CalcFactor` object will contain such data.  If not, please open an issue with Caesar.jl so that the necessary options may be added.

The `cfo` object contains the field `.factor::T` which is the type of the user factor being used, e.g. `myprior` from above example.  That is `cfo.factor::MyPrior`.  This is why `getSample` is using `rand(cfo.factor.Z)`.

`CalcFactor` was introduced in `IncrementalInference v0.20` to consolidate and standardize a variety of features that had previously been diseparate and unwieldy.

!!! tip
    Many factors already exists in `IncrementalInference`, `RoME`, and `Caesar`.  Please see their `src` directories for more details.


## Relative Factors on Manifolds
### One Dimension Example

Previously we looked at adding a prior.  This section demonstrates relative factors on manifold.  These are factors that introduce only relative information between variables in the factor graph.

Lets look at the Pose2Pose2 factor available in RoME as an example.  First, lets create the factor as before 
```julia
struct Pose2Pose2{T <: IIF.SamplableBelief} <: IIF.AbstractManifoldMinimize
  Z::T
end

DFG.getManifold(::Pose2Pose2) = Manifolds.SpecialEuclidean(2)
```

Extending the `getSample` method for our new `Pose2Pose2` factor is optional in this case.
The default behavior for sampling on `<:AbstractManifoldMinimize` factors defined on Group Manifolds is:
- The returned value from `getSample` represents a tangent vector at the identity element.
- The `SamplableBelief` shall be in the field `Z` and that shall be enough to fully define the factor.

If more advanced sampling is required, the `getSample` function can be extended. 

```julia
function getSample(cf::CalcFactor{<:Pose2Pose2}) 
  M = getManifold(cf.factor)
  ϵ = getPointIdentity(Pose2)
  X = sampleTangent(M, cf.factor.Z, ϵ)
  return X
end
```

The return type for `getSample` is unrestricted, and will be passed to the residual function "as-is".

!!! note
    Default dispatches in `IncrementalInference` will try use `cf.factor.Z` to `samplePoint` on manifold (for `<:AbstractPrior`) or `sampleTangent` (for `<:AbstractRelative`), which simplifies new factor definitions.  If, however, you wish to build more complicated sampling processes, then simply define your own `getSample(cf::CalcFactor{<:MyFactor})` function.

The selection of `<:IIF.AbstractManifoldMinimize`, akin to earlier `<:AbstractPrior`, instructs IIF to find the minimum of the provided residual function.
That is the one dimensional residual function, `residual = measurement - prediction`, is used during inference to approximate the convolution of conditional beliefs from the approximate beliefs of the connected variables in the factor graph.

The returned value (the factor measurement) from getSample will always be passed as the first argument (`X`) in the factor calculation.
The residual function should return a coordinate. 
```julia
function (cf::CalcFactor{<:Pose2Pose2})(X, p, q)
    M = getManifold(Pose2)
    q̂ = Manifolds.compose(M, p, exp(M, identity_element(M, p), X))
    Xc = vee(M, q, log(M, q, q̂))
    return Xc
end
```

!!! note
    Measurements and variables passed in to the factor residual function have related but potentially different types during construction or computation.  It is recommended to leave these incoming types unrestricted.  If you must define the types, make sure to allow sufficient dispatch freedom (i.e. dispatch to concrete types) and not force operations to "non-concrete" types.  Usage can be very case specific, and hence better to let Julia type-inference automation do the hard work of inferring the concrete types.

[//]: # (#TODO ### Advanced Sampling)

## Special Considerations

### Partial Factors

In some cases a factor only effects a partial set of dimensions of a variable.  For example a magnetometer being added onto a `Pose2` variable would look something like this:
```julia
struct MyMagnetoPrior{T<:SamplableBelief} <: AbstractPrior
  Z::T
  partial::Tuple{Int}
end

# define a helper constructor
MyMagnetoPrior(z) = MyMagnetoPrior(z, (3,))

getSample(cfo::CalcFactor{<:MyMagnetoPrior}) = samplePoint(cfo.factor.Z)
```

Similarly for `<:IIF.AbstractRelativeMinimize`, and note that the Roots version currently does not support the `.partial` option.

### Metadata

The MM-iSAMv2 algorithm relies on the Kolmogorov-Criteria as well as uncorrelated factor sampling.  This means that when generating fresh samples for a factor, those samples should not depend on values of variables in the graph or independent volatile variables.  That said, if you are comfortable or have a valid reason for introducing correlation between the factor sampling process with values inside the factor graph then you can do so via the `cfo.CalcFactor` interface.

At present `cfo` contains three main fields:
- `cfo.factor::MyFactor` the factor object as defined in the `struct` definition,
- `cfo.metadata::FactorMetadata`, which is currently under development and likely to change.
  - This contains references to the connected variables to the factor and more, and is useful for large data retrieval such as used in Terrain Relative Navigation (TRN).
- `cfo._sampleIdx` is the index of which computational sample is currently being calculated.


!!! note
    The old `.specialSampler` framework has been replaced with the standardized `::CalcFactor` interface.  See http://www.github.com/JuliaRobotics/IIF.jl/issues/467 for details.

## [OPTIONAL] Standardized Serialization

To take advantage of features like `DFG.saveDFG` and `DFG.loadDFG` a user specified type should be able to serialize via JSON standards.  The decision was taken to require bespoke factor types to always be converted into a JSON friendly `struct` which must be prefixed as type name with `PackedMyPrior{T}`.   Similarly, the user must also overload `Base.convert` as follows:
```julia
# necessary for overloading Base.convert
import Base: convert

struct PackedMyPrior <: PackedInferenceType
  Z::String
end

# IIF provides convert methods for `SamplableBelief` types
convert(::Type{PackedMyPrior}, pr::MyPrior{<:SamplableBelief}) = PackedMyPrior(convert(PackedSamplableBelief, pr.Z))
convert(::Type{MyPrior}, pr::PackedMyPrior) = MyPrior(IIF.convert(SamplableBelief, pr.Z))
```

Now you should be able to `saveDFG` and `loadDFG` your own factor graph types to Caesar.jl / FileDFG standard `.tar.gz` format.

```julia
fg = initfg()
addVariable!(fg, :x0, ContinuousScalar)
addFactor!(fg, [:x0], MyPrior(Normal()))

# generate /tmp/myfg.tar.gz
saveDFG("/tmp/myfg", fg)

# test loading the .tar.gz (extension optional)
fg2 = loadDFG("/tmp/myfg")

# list the contents
ls(fg2), lsf(fg2)
# should see :x0 and :x0f1 listed
```

### Factors supporting a Parametric Solution
See the [parametric solve section](@ref parametric_factors)

## Summary

All factors inherit from one of the following types, depending on their function:
* AbstractPrior: AbstractPrior are priors (unary factors) that provide an absolute constraint for a single variable. A simple example of this is an absolute GPS prior, or equivalently a (0, 0, 0) starting location in a [`Pose2`](@ref) scenario.
  * Requires: A getSample function
* IIF.AbstractRelativeMinimize: IIF.AbstractRelativeMinimize are relative factors that introduce an algebraic relationship between two or more variables. A simple example of this is an odometry factor between two pose variables, or a range factor indicating the range between a pose and another variable.
  * Requires: A getSample function and a residual function definition
  * The minimize suffix specifies that the residual function of this factor will be enforced by numerical minimization (find me the minimum of this function)
* IIF.AbstractRelativeRoots: IIF.AbstractRelativeRoots are relative factors that introduce algebraic relationships between two or more variables. They are the same as IIF.AbstractRelativeMinimize, however they use root finding to find the zero crossings (rather than numerical minimization).
  * Requires: A getSample function and a residual function definition

How do you decide which to use?
* If you are creating factors for world-frame information that will be tied to a single variable, inherit from `<:AbstractPrior`
  * GPS coordinates should be priors
* If you are creating factors for local-frame relationships between variables, inherit from IIF.AbstractRelativeMinimize
  * Odometry and bearing deltas should be introduced as pairwise factors and should be local frame
TBD: Users should start with IIF.AbstractRelativeMinimize, discuss why and when they should promote their factors to IIF.AbstractRelativeRoots.

!!! note
    IIF.AbstractRelativeMinimize does not imply that the overall inference algorithm only minimizes an objective function. The Multi-model iSAM algorithm is built around fixed-point analysis. Minimization is used here to locally enforce the residual function.

What you need to build in the new factor:
* A struct for the factor itself
* A sampler function to return measurements from the random ditributions
* If you are building a [`IIF.AbstractRelativeMinimize`](@ref) or a [`IIF.AbstractRelativeRoots`](@ref) you need to define a residual function to introduce the relative algebraic relationship between the variables
  * Minimization function should be lower-bounded and smooth
* A packed type of the factor which must be named Packed[Factor name], and allows the factor to be packed/transmitted/unpacked
* Serialization and deserialization methods
  * These are convert functions that pack and unpack the factor (which may be highly complex) into serialization-compatible formats
  * As the factors are mostly comprised of distributions (of type [`SamplableBelief`](@ref)), functions are provided to pack and unpack the distributions:
    * Packing: To convert from a [`SamplableBelief`](@ref) to a serializable obhect, use `convert(PackedSamplableBelief, ::SamplableBelief)`
    * Unpacking: To convert from string back to a `SamplableBelief`, use `convert(SamplableBelief, ::PackedSamplableBelief)`  

An example of this is the [`Pose2Point2BearingRange`](@ref), which provides a bearing+range relationship between a 2D pose and a 2D point.

```@docs
IIF.AbstractRelativeMinimize
IIF.AbstractRelativeRoots
```
