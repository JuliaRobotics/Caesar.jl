#  Multimodal incremental Smoothing and Mapping Algorithm

> **Work In Progress**

Placeholder for details on how the approximate sum-product inference algorithm ([mmiSAM](https://github.com/JuliaRobotics/IncrementalInference.jl)) works.  Until then, see [related literature](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/) for more details.

Algorithm combats the so called curse-of-dimensionality on the basis of eight principles outlined in the thesis work ["Multimodal and Inertial Sensor Solutions to Navigation-type Factor Graphs"](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Direct-References-1).

# Joint Probability

## General Factor Graph -- i.e. non-Gaussian and multi-modal

![mmfgbt](https://user-images.githubusercontent.com/6412556/52549388-db5f8b80-2da0-11e9-959c-4a8fe0890a87.gif)

## Inference on Bayes/Junction/Elimination Tree

See [tree solve video here](https://vimeo.com/332507701).

```@raw html
<a href="http://vimeo.com/332507701" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/52549397-e4505d00-2da0-11e9-958b-e9034c30477c.png" alt="Bayes/Junction tree example" width="640" border="0" /></a>
```

## Focussing Computation on Tree

### Incremental Updates

Recycling computations

### Fixed-Lag operation

Also mixed priority solving

### Federated Tree Solution (Multi session/agent)

Tentatively see [the multisession page](http://www.juliarobotics.org/Caesar.jl/latest/concepts/multisession/).

# Chapman-Kolmogorov (Belief Propagation / Sum-product)

The main computational effort is to focus compute cycles on dominant modes exhibited by the data, by dropping low likelihood modes (although not indefinitely) and not sacrificing accuracy individual major features. 

## Clique State Machine

The CSM is used to govern the inference process within a clique.  A [FunctionalStateMachine.jl](http://www.github.com/JuliaRobotics/FunctionStateMachine.jl) implementation is used to allow for initialization / incremental-recycling / fixed-lag solving, and will soon support federated branch solving as well as unidirectional message passing for fixed-lead operations.  See [the following video](https://vimeo.com/345576689) for an auto-generated---using `csmAnimate`---concurrent clique solving example.

## Mixture Parametric Method

Work In Progress -- deferred for progress on full functional methods, but likely to have Gaussian legacy algorithm with mixture model expansion added in the near future.

## Sequential Nested Gibbs Method

Current default inference method.

### Deterministic Convolution Approximation

Proposal distributions are computed using trust-region Newton methods through analytical or numerical factor definitions.

### Stochastic Product Approx of Infinite Functionals   

See mixed-manifold products presented in [the literature section](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Direct-References-1).

> writing in progress

## Full Deterministic Chapman-Kolmogorov Super Product Method

> Work in progress
