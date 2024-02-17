# Variable/Factor Considerations

A couple of important points:
* You **do not need to** modify or insert your new code into Caesar/RoME/IncrementalInference source code libraries -- they can be created and run anywhere on-the-fly!
* As long as the factors exist in the working space when the solver is run, the factors are automatically used -- this is possible due to Julia's [multiple dispatch design](https://docs.julialang.org/en/v1/manual/methods/index.html)
* Caesar.jl is designed to allow you to add new variables and factors to your own independent repository and incorporate them at will at compile-time or even run-time
* Residual function definitions for new factors types use a [callable struct (a.k.a functor) architecture](@ref custom_relative_factor) to simultaneously allow:  
  * Multiple dispatch (i.e. 'polymorphic' behavior)
  * Meta-data and in-place memory storage for advanced and performant code
  * An outside callback implementation style
* In most robotics scenarios, there is no need for new variables or factors:
  * Variables have various mechanisms that allow you to attach data to them, e.g. raw sensory data or identified April tags, so you do not need to create a new variable type just to store data
  * New variables are required only if you are representing a new state - TODO: Example of needed state
  * New factors are needed if:
    * You need to represent a constraint for a variable (known as a singleton) and that constraint type doesn't exist
    * You need to represent a constraint between two variables and that constraint type doesn't exist


All factors inherit from one of the following types, depending on their function:
* `AbstractPrior` is for priors (unary factors) that provide an absolute constraint for a single variable. A simple example of this is an absolute GPS prior, or equivalently a (0, 0, 0) starting location in a [`Pose2`](@ref) scenario.
  * Requires: A `getSample` function
* `AbstractRelativeMinimize` uses Optim.jl and is for relative factors that introduce an algebraic relationship between two or more variables. A simple example of this is an odometry factor between two pose variables, or a range factor indicating the range between a pose and another variable.
  * Requires: A `getSample` function and a residual function definition
  * The minimize suffix specifies that the residual function of this factor will be enforced by numerical minimization (find me the minimum of this function)
* [NEW] `AbstractManifoldMinimize` uses [Manopt.jl](https://github.com/JuliaManifolds/Manopt.jl).

How do you decide which to use?
* If you are creating factors for world-frame information that will be tied to a single variable, inherit from `<:AbstractPrior`
  * GPS coordinates should be priors
* If you are creating factors for local-frame relationships between variables, inherit from IIF.AbstractRelativeMinimize
  * Odometry and bearing deltas should be introduced as pairwise factors and should be local frame
TBD: Users should start with IIF.AbstractRelativeMinimize, discuss why and when they should promote their factors to IIF.AbstractRelativeRoots.

!!! note
    `AbstractRelativeMinimize` does not imply that the overall inference algorithm only minimizes an objective function. The MM-iSAM algorithm is built around fixed-point analysis. Minimization is used here to locally enforce the residual function.

What you need to build in the new factor:
* A struct for the factor itself
* A sampler function to return measurements from the random ditributions
* If you are building a `<:AbstractRelative` you need to define a residual function to introduce the relative algebraic relationship between the variables
  * Minimization function should be lower-bounded and smooth
* A packed type of the factor which must be named Packed[Factor name], and allows the factor to be packed/transmitted/unpacked
* Serialization and deserialization methods
  * These are convert functions that pack and unpack the factor (which may be highly complex) into serialization-compatible formats
  * As the factors are mostly comprised of distributions (of type `SampleableBelief`), while `JSON3.jl`` is used for serialization.
