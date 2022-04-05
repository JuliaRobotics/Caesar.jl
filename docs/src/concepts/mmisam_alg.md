#  Multimodal incremental Smoothing and Mapping Algorithm

!!! note
    Major refactoring of documentation under way 2020Q1.  Much of the previous text has be repositioned and being improved.  See references for details and check back here for updates in the coming weeks.

Caesar.jl uses an approximate sum-product inference algorithm ([mmiSAM](https://github.com/JuliaRobotics/IncrementalInference.jl)) works.  Until then, see [related literature](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/) for more details.

# Joint Probability

## General Factor Graph -- i.e. non-Gaussian and multi-modal

![mmfgbt](https://user-images.githubusercontent.com/6412556/52549388-db5f8b80-2da0-11e9-959c-4a8fe0890a87.gif)

## Inference on Bayes/Junction/Elimination Tree

See [tree solve video here](https://vimeo.com/332507701).

```@raw html
<a href="http://vimeo.com/332507701" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/52549397-e4505d00-2da0-11e9-958b-e9034c30477c.png" alt="Bayes/Junction tree example" width="640" border="0" /></a>
```

Algorithm combats the so called curse-of-dimensionality on the basis of eight principles outlined in the thesis work ["Multimodal and Inertial Sensor Solutions to Navigation-type Factor Graphs"](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Direct-References-1).

# Chapman-Kolmogorov (Belief Propagation / Sum-product)

The main computational effort is to focus compute cycles on dominant modes exhibited by the data, by dropping low likelihood modes (although not indefinitely) and not sacrificing accuracy individual major features. 

> D. Fourie, A. T. Espinoza, M. Kaess, and J. J. Leonard, “Characterizing marginalization and incremental operations on the Bayes tree,” in International Workshop on Algorithmic Foundations of Robotics (WAFR), 2020, submitted, under review.

## Focussing Computation on Tree

Link to new dedicated [Bayes tree pages](http://juliarobotics.org/Caesar.jl/latest/principles/bayestreePrinciples/).  The following sections describe different elements of clique recycling.

### Incremental Updates

Recycling computations similar to iSAM2, with option to complete future downward pass.

### Fixed-Lag operation (out-marginalization)

Active user (likely) computational limits on message passing.  Also mixed priority solving

### Federated Tree Solution (Multi session/agent)

Tentatively see [the multisession page](http://www.juliarobotics.org/Caesar.jl/latest/concepts/multisession/).

## Clique State Machine

The CSM is used to govern the inference process within a clique.  A [FunctionalStateMachine.jl](http://www.github.com/JuliaRobotics/FunctionStateMachine.jl) implementation is used to allow for initialization / incremental-recycling / fixed-lag solving, and will soon support federated branch solving as well as unidirectional message passing for fixed-lead operations.  See [the following video](https://vimeo.com/345576689) for an auto-generated---using `csmAnimate`---concurrent clique solving example.

## Sequential Nested Gibbs Method

Current default inference method.  See [Fourie et al., IROS 2016]

### Convolution Approximation (Quasi-Deterministic)

Convolution operations are used to implement the numerical computation of the probabilistic chain rule:
```math
P(A, B) = P(A | B)P(B)
```

Proposal distributions are computed by means of (analytical or numerical -- i.e. "algebraic") factor which defines a residual function:
```math
\delta : S \times \Eta \rightarrow \mathcal{R}
```
where ``S \times \Eta`` is the domain such that ``\theta_i \in S, \, \eta \sim P(\Eta)``, and ``P(\cdot)`` is a probability.

Please follow, a more detailed description is on [the convolutional computations page](https://juliarobotics.org/Caesar.jl/latest/principles/approxConvDensities/).

### Stochastic Product Approx of Infinite Functionals   

See mixed-manifold products presented in [the literature section](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Direct-References-1).

> writing in progress

## Mixture Parametric Method

Work In Progress -- deferred for progress on full functional methods, but likely to have Gaussian legacy algorithm with mixture model expansion added in the near future.

## Chapman-Kolmogorov

> Work in progress
