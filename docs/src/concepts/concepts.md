# Graph Concepts

A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables. Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below; see a broader discussion [in the related literature ](https://juliarobotics.org/Caesar.jl/latest/refs/literature/):

![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png)

## What are Variables and Factors

Factor graphs are bipartite, i.e. variables and factors.  The terminology of nodes and edges is reserved for actually storing the data on some graph-based technology.

Variables in the factor graph have not been observed, but we want to back them out given the observed values and algebra defining the structure between all observations.  Mathematically speaking, factors are actually "observed variables" that are stochastically "fixed".  Waving hands over the fact that factors encode both the algebraic model AND the observed measurement values.  If factors are constructed from statistically independent measurements (i.e. no direct correlations between measurements other than the known algebraic model that might connect them), then we can use Probabilistic Chain rule to write inference operation down (unnormalized):

```math
P(\Theta | Z)  \propto  P(Z | \Theta) P(\Theta)
```

This unnormalized "Bayes rule" is a consequence of two ideas, namely the [probabilistic chain rule](https://en.wikipedia.org/wiki/Chain_rule_%28probability%29) where Theta represents all variables and Z represents all measurements or data

```math
P(\Theta , Z) = P(Z | \Theta) P(\Theta)
```

or similarly,

```math
P(\Theta, Z) = P(\Theta | Z) P(Z).
```

Second, the uncorrelated measurement process assumption implies that `` P(Z) `` constant given the algebraic model.

