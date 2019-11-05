# Correspondence with Filtering?

A frequent discussion point is the correspondence between Kalman/particle/log-flow filtering strategies and factor graph formulations.  This section aims to shed light on the relationship, and to show that factor graph interpretations are a powerful generalization of existing filtering techniques.  The discussion follows a *build-your-own-filter* style and combines the [Approximate Convolution](https://www.juliarobotics.org/Caesar.jl/latest/principles/approxConvDensities/) and [Multiplying Densities](https://www.juliarobotics.org/Caesar.jl/latest/principles/multiplyingDensities/) pages as the required **prediction** and **update** cycle steps, respectively.  Using the steps described here, the user will be able to build fully-functional---i.e. non-Gaussian---(Bayes) filters.  

Other major topics are:  How out-marginalization works under a factor graph inference formulation, which is discussed separately as part of the [Bayes tree description page](https://www.juliarobotics.org/Caesar.jl/latest/principles/bayestreePrinciples/).  Second, see [FAQ on why do we even care about non-Gaussian](https://www.juliarobotics.org/Caesar.jl/latest/faq/#Why/Where-does-non-Gaussian-data-come-from?-1) signal processing.  The steps described on this page are accessible via several [multi-language interfaces (middleware)](https://www.juliarobotics.org/Caesar.jl/latest/concepts/multilang/#).

The conclusion at the end of the page tries to summarize why using a factor graph approach (w/ Bayes/junction tree inference) in a incremental/fixed-lag/federated sense---i.e. simultaneous localization and mapping (SLAM)---has merit.  The described steps forms part of the core operations used by the [multimodal incremental smoothing and mapping (mmisam)](https://www.juliarobotics.org/Caesar.jl/latest/concepts/mmisam_alg/) algorithm.

## The Target Tracking Problem (Conventional Filtering)

Consider a common example, two dimensional target tracking, where a projectile transits over a tracking station using various sensing technologies [[Zarchan 2013]](https://www.juliarobotics.org/Caesar.jl/latest/refs/literature/).  Position and velocity estimates of the target

### Prediction Step using a Factor Graph

Assume a constant velocity model from which the estimate will be updated through the measurement model described in the next section.  A constant velocity model is taken as (cartesian)
```math
\frac{dx}{dt} = 0 + \eta_x\\
\frac{dy}{dt} = 0 + \eta_y
```

or polar coordinates
```math
\frac{d\rho}{dt} = 0 + \eta_{\rho}\\
\frac{d\theta}{dt} = 0 + \eta_{\theta}.
```

In this example, noise is introduced as an affine slack variable `\eta`, but could be added as any part of the process model:
```math
\eta_j \sim p(...)
```

where `p` is any allowable noise probability density/distribution model -- discussed more in the next section.

After integration (assume zeroth order) the associated residual function can be constructed:
```math
\delta_i (\theta_{k}, \theta_{k-1}, \frac{d \theta_k}{dt}; \Delta t) = \theta_{k} - (\theta_{k-1} + \frac{d \theta_k}{dt} \Delta t)
```

Filter prediction steps are synonymous with a binary factor (conditional likelihood) between two variables where a prior estimate from one variable is projected (by means of a convolution) to the next variable.  The [convolutional principle page](https://www.juliarobotics.org/Caesar.jl/latest/principles/approxConvDensities/) describes a more detailed example on how a convolution can be computed.

!!! note

    Factor graphs are constructed along with the evolution of time which allows the mmisam inference algorithm to resolve variable marginal estimates both forward and backwards in time.  Conventional filtering only allows for forward-backward "smoothing" as two separate processes.  When inferring over a factor graph, all variables are factors are considered simultaneously according the topological connectivity irrespective of when and where which measurements were made or communicated -- as long as the factor graph (probabilistic model) captures the stochastics of the situation with sufficient accuracy.   

### Measurement Step using a Factor Graph

The measurement update is a product operation of infinite functional objects (probability densities)
```math
p(X_k | Z_a, Z_b) \approx p(X_k | X_{k-1}, Z_a) \times p(X_k | Z_b),
```

where `Z_.` represents conditional information for two beliefs on the same variable.  The product of the two functional estimates (beliefs) are multiplied by a stochastic algorithm described in more detail [on the multiplying functions page](https://www.juliarobotics.org/Caesar.jl/latest/principles/multiplyingDensities/).

Direct or indirect measurements of the state variables are should be modeled with the most sensible function `y = h(\theta, \eta)` -- i.e. linear or non-linear functions that approximate the underlying stochastics and physics of the process at hand.  

!!! note

    Mmisam allows for parametric, non-parametric, or intensity noise models which can be incorporated into any differentiable residual function.

Direct state observations can be added to the factor graph as prior factors directly on the variables.  Alternatively measurement models can be used to project belief through a measurement function:
```math
\delta_j(\theta_j, \eta_j) = y_j - h_j(\theta_j, \eta_j).
```

An illustration of both predictions (binary likelihood process model) and direct observations (measurements) is presented:

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/68166954-e45e4180-ff5b-11e9-91cb-0741d90a0c02.png" width="480" border="0" />
</p>
```

## Target Tracking (Beyond Filtering)

Consider a multi-sensory system along with data transmission delays, variable sampling rates, etc.;  when designing a filtering system to track one or multiple targets, it quickly becomes difficult to augment state vectors with the required state and measurement histories.  In contrast, the factor graph as a language allows for heterogeneous data streams to be combined in a common inference framework, and is [discussed further in the building distributed factor graphs section](http://www.juliarobotics.org/Caesar.jl/latest/concepts/building_graphs/).  

> TODO: Multi-modal (belief) vs. multi-hypothesis -- see thesis work on multimodal solutions in the mean time.
