# Caesar Concepts

A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion [here (author disclosure)](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1):
![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png).

This section discusses the various concepts in the Caesar framework.

## Getting Started with Caesar
The initial steps in constructing and solving graphs can be found in [Building and Solving Graphs](building_graphs.md).

We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).

## Visualization
Caesar supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualization.md)

## Extending Caesar
The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](adding_variables_factors.md).

## Connectivity and Extensibility
Caesar supports both in-memory solving (really fast, but for moderately-sized graphs) as well as database-driven solving (think massive graphs and multiple sessions). This is still under development/being refactored, and is discussed in [Common Data Persistence and Inference](database_interactions.md).

Although Caesar is Julia-based, it provides a ZMQ interface for extending it to other languages. This is discussed in [Extending Caesar via its ZMQ Interface](zmq.md).
