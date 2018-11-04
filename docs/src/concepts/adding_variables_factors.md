# Creating New Variables and Factors
In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications.

## Considerations
A couple of important points:
* You **do not need** to modify/fork/edit internal Caesar/RoME/IncrementalInference source code to introduce new  variable and factor types!
* As long as the factors exist in the working space when the solver is run, the factors are automatically used -- this is possible due to Julia's [multiple dispatch design](https://docs.julialang.org/en/v1/manual/methods/index.html)
* Caesar is designed to allow you to add new variables and factors to your own independent repository and incorporate them at will at compile-time or even run-time
* Residual function definitions for new factors types use a [callable struct (a.k.a functor) architecture](https://discourse.julialang.org/t/documenting-a-functor-callable-struct/8444) to simultaneously allow:  multiple dispatch (i.e. 'polymorphic' bahavior); meta data and in-place memory storage for advanced and performant code; as well as a outside callback implementation style.
* In most scenarios, there is no need for new variables or factors:
  * Variables have various mechanisms that allow you to attach data to them, e.g. raw sensory data or identified April tags, so you do not need to create a new variable type just to store data
  * New variables are required only if you are representing a new state - TODO: Example of needed state
  * New factors are needed if:
    * You need to represent a constraint for a variable (known as a singleton) and that constraint type doesn't exist
    * You need to represent a constraint between two variables and that constraint type doesn't exist

## Getting Started
We suggest the following design pattern for developing and building new factors:
1. You have reviewed the variable and factor types available in Caesar, RoME, and IncrementalInference and a new type is required - please see [Building Graphs](building_graphs.md) if you want to review what is currently available
1. Create a GitHub repository to store the new types
1. Create your new variable types
1. Create your new factor types
1. Implement unit tests to validate the correct operation of the types
1. Set up your solver to make use the custom types
1.1. This is much easier than it sounds
1. If the code is public and may be useful to the community, we ask if you could submit an issue against Caesar with information about the new types and the repository. Ideally we'd like to continually improve the core code and fold in community contributions.

The remainder of this section discusses each of these steps.

## Reviewing the Existing Types

## Creating a Repository
You can fork the following template repository to construct your own [Caesar Variable and Factor Examples](https://github.com/GearsAD/Caesar_VariableFactorExamples.jl.git).

## Creating New Variables

## Creating New Factors

## Unit Tests

## Using your Types with the Caesar Solver

## Contributing to Community
