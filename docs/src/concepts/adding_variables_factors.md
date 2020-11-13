# Creating New Variables and Factors
In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications.

!!! note
    Follow [JuliaRobotics/IncrementalInference.jl#1025](http://www.github.com/JuliaRobotics/IncrementalInference.jl#1025) for a reworking of `FactorMetadata` as a major piece of internal consolidation to simplify both this factor creation process, as well as many ohter library features.

## New Variable/Factor Considerations
A couple of important points:
* You **do not need to** modify/fork/edit internal Caesar/RoME/IncrementalInference source code to introduce new variable and factor types!
* As long as the factors exist in the working space when the solver is run, the factors are automatically used -- this is possible due to Julia's [multiple dispatch design](https://docs.julialang.org/en/v1/manual/methods/index.html)
* Caesar is designed to allow you to add new variables and factors to your own independent repository and incorporate them at will at compile-time or even run-time
* Residual function definitions for new factors types use a [callable struct (a.k.a functor) architecture](https://discourse.julialang.org/t/documenting-a-functor-callable-struct/8444) to simultaneously allow:  
  * Multiple dispatch (i.e. 'polymorphic' behavior)
  * Meta-data and in-place memory storage for advanced and performant code
  * An outside callback implementation style
* In most robotics scenarios, there is no need for new variables or factors:
  * Variables have various mechanisms that allow you to attach data to them, e.g. raw sensory data or identified April tags, so you do not need to create a new variable type just to store data
  * New variables are required only if you are representing a new state - TODO: Example of needed state
  * New factors are needed if:
    * You need to represent a constraint for a variable (known as a singleton) and that constraint type doesn't exist
    * You need to represent a constraint between two variables and that constraint type doesn't exist

## Getting Started
We suggest the following design pattern for developing and building new factors:
1. You have reviewed the variable and factor types available in Caesar, RoME, and IncrementalInference and a new type is required - please see [Building and Solving Graphs](https://www.juliarobotics.org/Caesar.jl/latest/concepts/available_varfacs/) if you want to review what is currently available
1. [OPTIONAL] Create a GitHub repository to store the new types (new types in the Julia Main scope is perfectly okay!)
1. Create your new variable types
1. Create your new factor types
1. Implement unit tests to validate the correct operation of the types
1. Set up your solver to make use the custom types
1.1. This is much easier than it sounds
1. If the code is public and may be useful to the community, we ask if you could submit an issue against Caesar with information about the new types and the repository. Ideally we'd like to continually improve the core code and fold in community contributions.

The remainder of this section discusses each of these steps.

## [OPTIONAL] Creating a Repository
You can fork the following template repository to construct your own [Caesar Variable and Factor Examples](https://github.com/GearsAD/Caesar_VariableFactorExamples.jl.git).

If this repository is going to be used for development of the new variables/factors as well as for the experiment (i.e. the code that builds the graph and solves it), you should probably start a simple end-to-end test that validates a basic version of your experimental setup (e.g. ):

```julia
#### This example is a basic test of the new variables and factors
#### that are added in this repo. The example is derived from
#### the hexagonal test example.

using Caesar, RoME
using Caesar_VariableFactorExamples # Your new variable/factor repository
# Using plotting for experiment validation
using RoMEPlotting

# 1. Init factor graph
#TODO

# 2. Add variables
#TODO

# 3. Add factors
# 3a. Add a new test prior
#TODO
# 3b. Add new types of odometry factors.
#TODO

# 4. Solve graph
solveTree!(fg)

# 5. Graph solution - assuming that you have this open in Atom.
drawPoses(fg)
```

## Creating New Variables

A handy macro can help define new variables:
```@docs
@defVariable
```

All variables have to derive from `IncrementalInference.InferenceVariable`.

What you need to build in the variable:
* `dims` - This is used during computation and defines the degrees of freedom (dimensions) for variable
* `labels` - This a required field, although it does not need to be populated. It consists of unique, indexable string identifiers, such as 'POSE', 'LANDMARK'. It assists with querying the data efficiently in large systems when using the database layer.  

You can then also add any additional fields that you would like to use for saving state information in variable. Note that these fields must be serializable as both JSON and Protobufs. Although you don't need to validate this, please keep the fields fairly simple and avoid complex structures with optional fields. TBD - provide a compatibility check for serialization and a docpage on it.

In a trivial example of Pose2:
* Our dimensions would then be 3: X, Y, theta
* The labels for Pose2 could be "POSE"

!!! note
    See [RoME.jl#244](http://www.github.com/JuliaRobotics/RoME.jl/issues/244) regarding plans to fundamentally integrate with [Manifolds.jl](http://www.github.com/JuliaManifolds/Manifolds.jl)

## Creating New Factors

All factors inherit from one of the following types, depending on their function:
* AbstractPrior: AbstractPrior are priors (unary factors) that provide an absolute constraint for a single variable. A simple example of this is an absolute GPS prior, or equivalently a (0, 0, 0) starting location in a [`Pose2`](@ref) scenario.
  * Requires: A getSample function
* AbstractRelativeFactorMinimize: AbstractRelativeFactorMinimize are relative factors that introduce an algebraic relationship between two or more variables. A simple example of this is an odometry factor between two pose variables, or a range factor indicating the range between a pose and another variable.
  * Requires: A getSample function and a residual function definition
  * The minimize suffix specifies that the residual function of this factor will be enforced by numerical minimization (find me the minimum of this function)
* AbstractRelativeFactor: AbstractRelativeFactor are relative factors that introduce algebraic relationships between two or more variables. They are the same as AbstractRelativeFactorMinimize, however they use root finding to find the zero crossings (rather than numerical minimization).
  * Requires: A getSample function and a residual function definition

How do you decide which to use?
* If you are creating factors for world-frame information that will be tied to a single variable, inherit from AbstractPrior
  * GPS coordinates should be priors
* If you are creating factors for local-frame relationships between variables, inherit from AbstractRelativeFactorMinimize
  * Odometry and bearing deltas should be introduced as pairwise factors and should be local frame
TBD: sUsers **should** start with AbstractRelativeFactorMinimize, discuss why and when they should promote their factors to AbstractRelativeFactor.

> Note: AbstractRelativeFactorMinimize does not imply that the overall inference algorithm only minimizes an objective function. The Multi-model iSAM algorithm is built around fixed-point analysis. Minimization is used here to locally enforce the residual function.

What you need to build in the new factor:
* A struct for the factor itself
* A sampler function to return measurements from the random ditributions
* If you are building a [`AbstractRelativeMinimize`](@ref) or a [`AbstractRelativeRoots`](@ref) you need to define a residual function to introduce the relative algebraic relationship between the variables
  * Minimization function should be lower-bounded and smooth
* A packed type of the factor which must be named Packed[Factor name], and allows the factor to be packed/transmitted/unpacked
* Serialization and deserialization methods
  * These are convert functions that pack and unpack the factor (which may be highly complex) into serialization-compatible formats
  * As the factors are mostly comprised of distributions (of type [`SamplableBelief`](@ref)), functions are provided to pack and unpack the distributions:
    * Packing: To convert from a [`SamplableBelief`](@ref) to a string, use `string(::SamplableBelief)::String`
    * Unpacking: To convert from string back to a `SamplableBelief`, use `extractdistribution(::String)::SamplableBelief`  

An example of this is the [`Pose2Point2BearingRange`](@ref), which provides a bearing+range relationship between a 2D pose and a 2D point.

```@docs
AbstractRelativeMinimize
AbstractRelativeRoots
```

### Pose2Point2BearingRange Struct

```julia
mutable struct Pose2Point2BearingRange{B <: IIF.SamplableBelief, R <: IIF.SamplableBelief} <: IncrementalInference.AbstractRelativeFactor
    bearing::B
    range::R
    Pose2Point2BearingRange{B,R}() where {B,R} = new{B,R}()
    Pose2Point2BearingRange{B,R}(x1::B,x2::R) where {B <: IIF.SamplableBelief,R <: IIF.SamplableBelief} = new{B,R}(x1,x2)
end
# Convenient constructor
Pose2Point2BearingRange(x1::B,x2::R) where {B <: IIF.SamplableBelief,R <: IIF.SamplableBelief} = Pose2Point2BearingRange{B,R}(x1,x2)
```

### Pose2Point2BearingRange Sampler

```julia
# Return N samples from the two distributions
function getSample(pp2br::Pose2Point2BearingRange, N::Int=1)
  smpls = zeros(2, N)
  smpls[1,:] = rand(pp2br.bearing, N)[:]
  smpls[2,:] = rand(pp2br.range, N)[:]
  return (smpls,)
end
```

### Pose2Point2BearingRange Residual Function (Functor)

```julia
# define the conditional probability constraint
function (pp2br::Pose2Point2BearingRange)(res::Array{Float64},
        userdata::FactorMetadata,
        idx::Int,
        meas::Tuple{Array{Float64,2}},
        xi::Array{Float64,2},
        lm::Array{Float64,2} )
  #
  res[1] = lm[1,idx] - (meas[1][2,idx]*cos(meas[1][1,idx]+xi[3,idx]) + xi[1,idx])
  res[2] = lm[2,idx] - (meas[1][2,idx]*sin(meas[1][1,idx]+xi[3,idx]) + xi[2,idx])
  nothing
end
```

### Pose2Point2BearingRange Packing and Unpacking

The packing structure:

```julia
mutable struct PackedPose2Point2BearingRange <: IncrementalInference.PackedInferenceType
    bearstr::String
    rangstr::String
    PackedPose2Point2BearingRange() = new()
    PackedPose2Point2BearingRange(s1::AS, s2::AS) where {AS <: AbstractString} = new(string(s1),string(s2))
end
```

The packing and unpacking converters (note the use of `string` and `extractdistribution`):

```julia
function convert(::Type{PackedPose2Point2BearingRange}, d::Pose2Point2BearingRange{B, R}) where {B <: IIF.SamplableBelief, R <: IIF.SamplableBelief}
  return PackedPose2Point2BearingRange(string(d.bearing), string(d.range))
end

function convert(::Type{Pose2Point2BearingRange}, d::PackedPose2Point2BearingRange)
 # where {B <: IIF.SamplableBelief, R <: IIF.SamplableBelief}
  Pose2Point2BearingRange(extractdistribution(d.bearstr), extractdistribution(d.rangstr))
end
```

## Unit Tests

What you should test:
* Creation of the factor
* Sampling of the factor
* Residual testing
* Solving using the variables and factors
* Serialization and deserialization

An example of these tests can be seen for the trivial case shown in the example repo [ExamplePrior Unit Tests](https://github.com/GearsAD/Caesar_VariableFactorExamples.jl/blob/master/test/ExamplePrior.jl).

## Using your Types with the Caesar Solver

As above, as long as you bring your factors into the workspace, you should be able to use them in your experimental setup.

You can validate this with the existence check code in [Building and Solving Graphs](@ref).

> Note: This has been made available as `IncrementalInference.getCurrentWorkspaceVariables()` and `IncrementalInference.getCurrentWorkspaceFactors()`in IncrementalInference v0.4.4.

## Contributing back to the Community
Consider contributioning back, so if you have developed variables and factors that may be useful to the community, please write up an issue in [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) or submit a PR to the relavent repo.  Note also that work is ongoing to simplify and consolidate the code structure given all previously known feature requests and requirements.
