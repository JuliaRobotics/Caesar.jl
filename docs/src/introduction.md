# Introduction
Caesar is an open-source robotic software stack aimed at localization and mapping for robotics, using non-Gaussian graphical model state-estimation techniques.  Ths factor graph method is well suited to combining heterogeneous and ambiguous sonsor data streams.  The focus is predominantly on geometric/spatial/semantic estimation tasks related to simultaneous localization and mapping (SLAM).  The software is also highly extensible and well suited to a variety of estimation/filtering-type tasks — especially in non-Gaussian/multimodal settings.  

Caesar.jl addresses numerous issues that arise in prior SLAM solutions, including: 
- solving under-defined systems, 
- inference with non-Gaussian measurements, 
- standard features for natively handling ambiguous data association and multi-hypotheses, 
- simplifying bespoke factor development, 
- centralized (or peer-to-peer distributed) factor-graph persistence, 
- federated multi-session/agent reduction, and many more.  

Caesar.jl is community orientef and originates from research work at MIT, see the [literature reference page](https://www.juliarobotics.org/Caesar.jl/latest/refs/literature/) for more information.

[NavAbility](https://www.navability.io/) administers the Caesar/RoME/IncrementalInference packages, please contact [info@navability.io](https://navability.io/contact/) for more details.

Ongoing work on this project focuses on the open development of a stable, reliable, verified, user-friendly, and growing library that is well suited to various data-fusion / state-estimation aspects of robotics and autonomy in [non-Gaussian/multi-modal](https://juliarobotics.org/Caesar.jl/latest/concepts/concepts/#Why/Where-does-non-Gaussian-data-come-from?-1) data processing.

# A Few Highlights

The Caesar framework offers these and other features:
* Distributed Factor Graph representation deeply-coupled with an on-Manifold probabilistic algebra language;
* Localization using [MM-iSAMv2](http://www.juliarobotics.org/Caesar.jl/latest/refs/literature/#Related-Literature-1)
  * Multi-core inference supporting `Pose2, Pose3, Point2, Point3, Multi-modal (multi-hypothesis), IMU preintegration, KDE density, intensity map, partial constraints, null hypothesis, etc`.
* Native multi-modal (hypothesis) representation in the factor-graph, see [Data Association and Hypotheses](@ref):
  * Multi-modal and non-parametric representation of constraints;
  * Gaussian distributions are but one of the many representations of measurement error;
* Out-of-library extendable for [Creating New Variables and Factors](@ref);
* Natively supports legacy Gaussian parametric and max-mixtures solutions;
* Local in-memory solving on the device as well as database-driven centralized solving (micro-service architecture);
* Natively support *Clique Recycling* (i.e. fixed-lag out-marginalization) for continuous operation as well as off-line batch solving, see more at [Using Incremental Updates (Clique Recycling I)](@ref);
* Natively supports [Dead Reckon Tethering](examples/deadreckontether.md);
* Natively supports Federated multi-session/agent solving;
* Native support for `Entry=>Data` blobs [for storing large format data](https://juliarobotics.org/Caesar.jl/latest/concepts/entry_data/).
* Middleware support, e.g. see the [ROS Integration Page](examples/using_ros.md).
