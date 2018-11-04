# Creating New Variables and Factors

Discuss design considerations for creating variables and factors.

A couple of important points:
* You **do not need** to modify/fork/edit internal Caesar/RoME/IncrementalInference source code!
* As long as the factors exist in the working space of the solver, the factors are implicitly and automatically used -- possible and tractible due to Julia's [multiple dispatch design](https://docs.julialang.org/en/v1/manual/methods/index.html).
* Caesar is designed to allow you to add new variables and factors to your own independent repository and incorporate them at will at compile or even run-time.
* Residual function definitions for new factors types use a [callable struct (a.k.a functor) architecture](https://discourse.julialang.org/t/documenting-a-functor-callable-struct/8444) to simultaneously allow:  multiple dispatch (i.e. 'polymorphic' bahavior); meta data and in-place memory storage for advanced and performant code; as well as a outside callback implementation style.

## A Template Repository

You can fork the following template repository to construct your own [Caesar Template Repository](https://github.com/GearsAD/Caesar_NewFactorTemplate.jl.git).

## Creating New Variables

## Creating New Factors
