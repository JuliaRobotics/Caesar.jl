# Creating New Variables and Factors
In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications.

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
plotSLAM2DPoses(fg)
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

!!! note
    This has been made available as `IncrementalInference.getCurrentWorkspaceVariables()` and `IncrementalInference.getCurrentWorkspaceFactors()`.

## Contributing back to the Community
Consider contributioning back, so if you have developed variables and factors that may be useful to the community, please write up an issue in [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) or submit a PR to the relavent repo.  Note also that work is ongoing to simplify and consolidate the code structure given all previously known feature requests and requirements.
