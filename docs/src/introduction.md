# Introduction

Caesar is an open-source robotic software stack aimed at localization and mapping for robotics, using non-Gaussian graphical model state-estimation techniques.  Ths factor graph method is well suited to combining heterogeneous and ambiguous sonsor data streams.  The focus is predominantly on geometric/spatial/semantic estimation tasks related to simultaneous localization and mapping (SLAM).  The software is also highly extensible and well suited to a variety of estimation /filtering-type tasks — especially in non-Gaussian/multimodal settings.  Check out a brief description on why [non-Gaussian / multi-modal](https://juliarobotics.org/Caesar.jl/latest/concepts/concepts/#Why/Where-does-non-Gaussian-data-come-from?-1) data processing needs arise.

# A Few Highlights

Caesar.jl addresses numerous issues that arise in prior SLAM solutions, including: 
* Distributed Factor Graph representation deeply-coupled with an on-Manifold probabilistic algebra language;
* Localization using different algorithms:
  * [MM-iSAMv2](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Related-Literature-1)
  * Parametric methods, including regular Guassian or Max-Mixtures.
  * Other multi-parametric and non-Gaussian algorithms are presently being implemented.
* Solving under-defined systems, 
* Inference with non-Gaussian measurements, 
* Standard features for natively handling ambiguous data association and multi-hypotheses, 
  * Native multi-modal (hypothesis) representation in the factor-graph, see [Data Association and Hypotheses](@ref):
  * Multi-modal and non-parametric representation of constraints;
  * Gaussian distributions are but one of the many representations of measurement error;
* Simplifying bespoke factor development, 
* Centralized (or peer-to-peer decentralized) factor-graph persistence, 
  * i.e. Federated multi-session/agent reduction.
* Multi-CPU inference.
* Out-of-library extendable for [Creating New Variables and Factors](@ref);
* Natively supports legacy Gaussian parametric and max-mixtures solutions;
* Local in-memory solving on the device as well as database-driven centralized solving (micro-service architecture);
* Natively support *Clique Recycling* (i.e. fixed-lag out-marginalization) for continuous operation as well as off-line batch solving, see more at [Using Incremental Updates (Clique Recycling I)](@ref);
* Natively supports [Dead Reckon Tethering](examples/deadreckontether.md);
* Natively supports Federated multi-session/agent solving;
* Native support for `Entry=>Data` blobs [for storing large format data](https://juliarobotics.org/Caesar.jl/latest/concepts/entry_data/).
* Middleware support, e.g. see the [ROS Integration Page](examples/using_ros.md).
