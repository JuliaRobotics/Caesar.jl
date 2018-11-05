# Creating New Variables and Factors
In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications.

## Considerations
A couple of important points:
* You **do not need** to modify/fork/edit internal Caesar/RoME/IncrementalInference source code to introduce new  variable and factor types!
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
1. You have reviewed the variable and factor types available in Caesar, RoME, and IncrementalInference and a new type is required - please see [Building and Solving Graphs](building_graphs.md) if you want to review what is currently available
1. Create a GitHub repository to store the new types
1. Create your new variable types
1. Create your new factor types
1. Implement unit tests to validate the correct operation of the types
1. Set up your solver to make use the custom types
1.1. This is much easier than it sounds
1. If the code is public and may be useful to the community, we ask if you could submit an issue against Caesar with information about the new types and the repository. Ideally we'd like to continually improve the core code and fold in community contributions.

The remainder of this section discusses each of these steps.

## Reviewing the Existing Types
Please see [Building and Solving Graphs](building_graphs.md) to review what variables and factors are currently supported.

## Creating a Repository
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
batchSolve!(fg)

# 5. Graph solution - assuming that you have this open in Atom.
drawPoses(fg)
```

## Creating New Variables

All variables have to derive from `IncrementalInference.InferenceVariable`.

What you need to build in the variable:
* `dims` - This is used during computation and defines the degrees of freedom (dimensions) for variable
* `labels` - This a required field, although it does not need to be populated. It consists of unique, indexable string identifiers, such as 'POSE', 'LANDMARK'. It assists with querying the data efficiently in large systems when using the database layer.  

You can then also add any additional fields that you would like to use for saving state information in variable. Note that these fields **must** be serializable as both JSON and Protobufs. Although you don't need to validate this, please keep the fields fairly simple and avoid complex structures with optional fields. TBD - provide a compatibility check for serialization and a docpage on it.

In a trivial example of Pose2:
* Our dimensions would then be 3 - X, Y, theta
* The labels for Pose2 could be "POSE"

## Creating New Factors

All factors inherit from one of the following types, depending on their function:
* FunctorSingleton: FunctorSingletons are priors (unary factors) that provide an absolute constraint for a single variable. A simple example of this is an absolute GPS prior, or equivalently a (0, 0, 0) starting location in a Pose2D scenario.
  * Requires: a getSample function
* FunctorPairwiseMinimize: FunctorPairwiseMinimize are relative factors that introduce an algebraic relationship between two or more variables. A simple example of this is an odometry factor between two pose variables, or a range factor indicating the range between a pose and another variable.
  * Requires: a getSample function and a residual function definition
  * The minimize suffix specifies that the residual function of this factor will be enforced by numerical minimization (find me the minimum of this function)
* FunctorPairwise: FunctorPairwise are relative factors that introduce algebraic relationships between two or more variables. They are the same as FunctorPairwiseMinimize, however they use root finding to find the zero crossings (rather than numerical minimization).
  * Requires: getSample function and a residual function definition
TBD: Users **should** start with FunctorPairwiseMinimize, discuss why and when they should promote their factors to FunctorPairwise.

> Note: FunctorPairwiseMinimize does not imply that the overall inference algorithm only minimizes an objective function. The Multi-model iSAM algorithm is built around fixed-point analysis. Minimization is used here to locally enforce the residual function.

What you need to build in the factor:
* A struct for the factor itself
* A sampler: The sampler
* A residual function: You introduce a relative algebraic relationship between the two or more variables
  * Does not need to be signed
  * Minimization function that should be lower-bounded and smooth
* Serialization and deserialization methods


> Note: Remember introduce absolute information (global reference frame) information via priors, and relative (local reference frame) information via pairwise. That means that GPS coordinates should be priors, and differences in coordinates should be pairwise factors. Similarly, odometry and bearing deltas should be introduced as pairwise factors.

## Unit Tests

What you need to test:
* Creation of the factor
* Sampling of the factor
* Residual testing
* Solving using the variables and factors
* Serialization and deserialization

## Using your Types with the Caesar Solver

As above, as long as you bring your factors into the workspace, you should be able to use them in your experimental setup.

You can validate this with the code from [Building and Solving Graphs](building_graphs.md). Once you have imported Caesar and your library into the workspace, you can check whether the new types exist by running the following:

For variables:
```julia
using RoME, Caesar
subtypes(IncrementalInference.InferenceVariable)
```

> Note: This has been made available as `IncrementalInference.getCurrentWorkspaceVariables()` in IncrementalInference v0.4.4.

For factors:
```julia
using RoME, Caesar
println("- Singletons (priors): ")
println.(sort(string.(subtypes(IncrementalInference.FunctorSingleton))));
println("- Pairwise (variable constraints): ")
println.(sort(string.(subtypes(IncrementalInference.FunctorPairwise))));
println("- Pairwise (variable minimization constraints): ")
println.(sort(string.(subtypes(IncrementalInference.FunctorPairwiseMinimize))));
```

> Note: This has been made available as `IncrementalInference.getCurrentWorkspaceFactors()` in IncrementalInference v0.4.4.

## Contributing to Community
We really appreciate any contributions, so if you have developed variables and factors that may be useful to the community, please write up an issue in [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) with a link to your repo and a short description of the use-case(s).
