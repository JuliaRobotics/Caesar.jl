# Data Association and Hypotheses

Ambiguous data and processing often produce complicated data association situations.  In SLAM, loop-closures are a major source of concern when developing autonomous subsystems or behaviors.  In conventional parametric Gaussian-only systems an incorrect loop-closure can occur unpredictably cause highly unstable numerical solutions.  The mm-iSAM algorithm was conceived to directly address these (and other related) issues by changing the fundamental manner in which the statistical inference is performed (see literature section for links).

Data association applies well beyond just loop-closure, including but not limited to navigation-affordance matching and discrepancy detection, and indicates the versatility of the IncrementalInference.jl standardized `multihypo` interface.  Note that much more is possible, however, the so-called single-fraction multihypo approach already yields significant benefits and simplicity.

## Illustration

Consider the following canonical illustrations regarding feature selection on some domain (say landmarks in geometry space):

<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/76276600-2686ef00-627e-11ea-9c86-fd21852ef793.png" width="640" border="0" />
</p>

## Modeling Multihypothesis

The `addFactor!` function offers the `multihypo=[1;0.6;0.3;0.1;1]` keyword option, where the user can convert any one variable of an arbitrary factor into a fractional list of probabilities -- e.g. a three variable factor now has triple association uncertainty on the middle variable.  A more classical binary multihypothesis example is illustated in the multimodal (non-Gaussian) factor graph below:

<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/76276833-dfe5c480-627e-11ea-9d84-2df1e1138bbf.png" width="640" border="0" />
</p>
