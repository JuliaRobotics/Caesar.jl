# Caesar Concepts

A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion [here (author disclosure)](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1):

![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png).

## What are Variables and Factors

Factor graphs are bipartite, i.e. variables and factors.  In practice we use "nodes" to represent both variables and factors with edges between.  In future, we will remove the wording "node" from anything Factor Graph usage/abstraction related (only vars and factors).  Nodes and edges will be used as terminology for actually storing the data on some graph storage/process foundation technology.

Even more meta -- factors are "variables" that have already been observed and are now stochastically "fixed".  Waving hands over the fact that a factors encode both the algebraic model AND the observed measurement values.

Variables in the factor graph have not been observed, but we want to back them out from the observed values and algebra relating them all.  If factors are constructed from statistically independent measurements (i.e. no direct correlations between measurements other than the algebra already connecting them), then we can use Probabilistic Chain rule to write inference operation down (unnormalized):

```math
P(\Theta | Z)  =  P(Z | \Theta) P(\Theta)
```

where Theta represents all variables and Z represents all measurements or data, and

```math
P(\Theta , Z) = P(Z | \Theta) P(\Theta) ``

or

```math
P(\Theta, Z) = P(\Theta | Z) P(Z).
```

You'll notice the first looks like "Bayes rule" and we take `` P(MEAS) `` as a constant (the uncorrelated assumption).

# Why/Where does non-Gaussian data come from?

Gaussian error models in measurement or data cues will only be Gaussian (normally distributed) if all physics/decisions/systematic-errors/calibration/etc. has a correct algebraic model in every single circumstance.  Caesar.jl and mm-iSAM is heavily focussed on state-estimation from a plethora of heterogenous data.  Four major categories of non-Gaussian errors have thus far been considered:
- Uncertain decisions (a.k.a. data association), such as a robot trying to decide if a navigation loop-closure can be deduced from a repeat observation of a similar object or measurement from current and past data.  These issues are commonly also referred to as multi-hypothesis.
- Under-determined or under-defined systems where there are more variables than constraining measurements to fully define the system as a single mode---a.k.a solution ambiguity.  For example, in 2D consider two range measurements resulting in two possible locations through trilateration.
- Nonlinearity.  For example in 2D, consider a Pose2 odometry where the orientation is uncertain:  The resulting belief of where a next pose might be (convolution with odometry factor) results in a banana shape curve, even though the entire process is driven by assumed Gaussian belief.
- Physics of the measurement process.  Many, if not all measurement processes exhibit non-Gaussian behaviour.  For example, acoustic/radio time-of-flight measurements, using either pulse-train or matched filtering, result in an "energy intensity" over time/distance of what the range to a scattering-target/source might be--i.e. highly non-Gaussian.

# Getting Started with Caesar

This section discusses the various concepts in the Caesar framework.

The initial steps in constructing and solving graphs can be found in [Building and Solving Graphs](building_graphs.md).

We also recommend reviewing the various examples available in the [Examples section](../examples/examples.md).

## Visualization
Caesar supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in [Visualization with Arena.jl and RoMEPlotting.jl](arena_visualizations.md)

## Extending Caesar
The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in [Creating New Variables and Factors](adding_variables_factors.md).

## Connectivity and Extensibility
Caesar supports both in-memory solving (really fast, but for moderately-sized graphs) as well as database-driven solving (think massive graphs and multiple sessions). This is still under development/being refactored, and is discussed in [Common Data Persistence and Inference](database_interactions.md).

Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in [Caesar Multi-Language Support](multilang.md).
