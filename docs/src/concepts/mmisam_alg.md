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

## Sequential Nested Gibbs Method

Current default inference method.

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

A trust-region, nonlinear gradient decent method is used to enforce the residual function ``\delta (\theta_S)`` in a leave-one-out-Gibbs strategy for all the factors and variables in each clique.  Each time a factor residual is enforced for another particle along with a sample from the stochastic noise term.  Solutions are found either through root finding on "full dimension" equations ([source code here](https://github.com/JuliaRobotics/IncrementalInference.jl/blob/62afec6300c899d567be29b06f8d9b0919b31878/src/SolverUtilities.jl#L128)):
```math
\text{solve}_{\theta_i} ~ s.t. \, 0 = \delta(\theta_{S}; \eta)
```
Or minimization of "low dimension" equations ([source code here](https://github.com/JuliaRobotics/IncrementalInference.jl/blob/62afec6300c899d567be29b06f8d9b0919b31878/src/SolverUtilities.jl#L72)) that might not have any roots in ``\theta_i``:
```math
\text{argmin}_{\theta_i} ~ [\delta(\theta_{S}; \eta)]^2
```

> Gradient decent methods are obtained from the Julia Package community, namely [NLsolve.jl](https://github.com/JuliaNLSolvers/NLsolve.jl) and [Optim.jl](https://github.com/JuliaNLSolvers/Optim.jl).

The factor noise term can be any samplable belief (a.k.a. [`IIF.SamplableBelief`](https://github.com/JuliaRobotics/IncrementalInference.jl/blob/2b9f1c3d03e796bc24fbcc622329769dadd94288/src/DefaultNodeTypes.jl#L3)), either through algebraic modeling, or (**critically**) directly from the sensor measurement that is driven by the underlying physics process.  Parametric factors ([Distributions.jl](https://github.com/JuliaStats/Distributions.jl)) or direct physical measurement noise can be used via `AliasingScalarSampler` or `KernelDensityEstimate`.

This figure shows an example of the quasi-deterministic convolution of green and red functions to produce the black trace as proposal distribution 
```@raw html
<a href="https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/61175404-3b4f9d80-a59e-11e9-85db-ca6bbdb73ffd.png" alt="Bayes/Junction tree example" width="640" border="0" /></a>
```

> Also see [1.2], Chap. 5, Approximate Convolutions.

### Stochastic Product Approx of Infinite Functionals   

See mixed-manifold products presented in [the literature section](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Direct-References-1).

> writing in progress

## Mixture Parametric Method

Work In Progress -- deferred for progress on full functional methods, but likely to have Gaussian legacy algorithm with mixture model expansion added in the near future.

## Full Deterministic Chapman-Kolmogorov Super Product Method

> Work in progress, likely to include Kernel Embedding and Homotopy Continuation methods for combining convolution and product operations as a concurrent calculation.
