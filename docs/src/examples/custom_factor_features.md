# Custom Factor Features

## Contributing back to the Community

Consider contributioning back, so if you have developed variables and factors that may be useful to the community, please write up an issue in [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) or submit a PR to the relavent repo.

## [What is `CalcFactor`](@id whatiscalcfactor)

`CalcFactor` is part of the `IIF` interface to all factors.  It contains metadata and other important bits of information that are useful in a wide swath of applications.  As work requires more interesting features from the code base, it is likely that the `cfo::CalcFactor` object will contain such data.  If not, please open an issue with Caesar.jl so that the necessary options may be added.

The `cfo` object contains the field `.factor::T` which is the type of the user factor being used, e.g. `myprior` from above example.  That is `cfo.factor::MyPrior`.  This is why `getSample` is using `rand(cfo.factor.Z)`.

`CalcFactor` was introduced in `IncrementalInference v0.20` to consolidate and standardize a variety of features that had previously been diseparate and unwieldy.

!!! tip
    Many factors already exists in `IncrementalInference`, `RoME`, and `Caesar`.  Please see their `src` directories for more details.
### Factor Metadata

The MM-iSAMv2 algorithm relies on the Kolmogorov-Criteria as well as uncorrelated factor sampling.  This means that when generating fresh samples for a factor, those samples should not depend on values of variables in the graph or independent volatile variables.  That said, if you are comfortable or have a valid reason for introducing correlation between the factor sampling process with values inside the factor graph then you can do so via the `cfo.CalcFactor` interface.

At present `cfo` contains three main fields:
- `cfo.factor::MyFactor` the factor object as defined in the `struct` definition,
- `cfo.metadata::FactorMetadata`, which is currently under development and likely to change.
  - This contains references to the connected variables to the factor and more, and is useful for large data retrieval such as used in Terrain Relative Navigation (TRN).
- `cfo._sampleIdx` is the index of which computational sample is currently being calculated.


!!! note
    The old `.specialSampler` framework has been replaced with the standardized `::CalcFactor` interface.  See http://www.github.com/JuliaRobotics/IIF.jl/issues/467 for details.

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

## Factors supporting a Parametric Solution
See the [parametric solve section](@ref parametric_factors)

# [Standardized Factor Serialization](@id factor_serialization)

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
