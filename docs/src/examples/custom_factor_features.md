# Custom Factor Features

## Partial Factors

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

## Factor Metadata

The MM-iSAMv2 algorithm relies on the Kolmogorov-Criteria as well as uncorrelated factor sampling.  This means that when generating fresh samples for a factor, those samples should not depend on values of variables in the graph or independent volatile variables.  That said, if you are comfortable or have a valid reason for introducing correlation between the factor sampling process with values inside the factor graph then you can do so via the `cfo.CalcFactor` interface.

At present `cfo` contains three main fields:
- `cfo.factor::MyFactor` the factor object as defined in the `struct` definition,
- `cfo.metadata::FactorMetadata`, which is currently under development and likely to change.
  - This contains references to the connected variables to the factor and more, and is useful for large data retrieval such as used in Terrain Relative Navigation (TRN).
- `cfo._sampleIdx` is the index of which computational sample is currently being calculated.


!!! note
    The old `.specialSampler` framework has been replaced with the standardized `::CalcFactor` interface.  See http://www.github.com/JuliaRobotics/IIF.jl/issues/467 for details.

## [Standardized Factor Serialization](@id factor_serialization)

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
