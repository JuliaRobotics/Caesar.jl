# Multisession Operation

Having all the data consolidated in a factor graph allows us to do something we find really exciting: reason against data for different robots, different robot sessions, even different users. Of course, this is all optional, and must be explicitly configured, but if enabled, current inference solutions can make use of historical (or other robot) data to improve continually improve their solutions.

Consider a single robot working in a common environment that has driven around the same area three times and has identified a landmark that is probably the same. We can automatically close the loop and use the information from the prior data to improve our current solution.

To do this, when we configure the robot we specify that the sessions are operating in a shared environment, e.g 'lab'. If, in 

For example, if we identify a simple landmark `l1` in three graphs:
[]()


A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion [here (author disclosure)](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1):
![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png).

This section discusses the various concepts in the Caesar framework.

## Getting Started with Caesar
The initial steps in constructing and solving graphs can be found in [Building and Solving Graphs](building_graphs.md).

We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).

## Visualization
Caesar supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md)

## Extending Caesar
The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](adding_variables_factors.md).

## Connectivity and Extensibility
Caesar supports both in-memory solving (really fast, but for moderately-sized graphs) as well as database-driven solving (think massive graphs and multiple sessions). This is still under development/being refactored, and is discussed in [Common Data Persistence and Inference](database_interactions.md).

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).
