# Examples

## Basics
The following examples demonstrate the conceptual operation of Caesar, highlighting specific features of the framework and its use.

### Continuous Scalar

This abstract example illustrates how `IncrementalInference` enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information.

[Continuous Scalar Example](basic_continuousscalar.md)

### Hexagonal 2D
A simple 2D robot trajectory example is expanded below using techniques developed in simultaneous localization and mapping (SLAM).

[Hexagonal 2D Example](basic_hexagonal2d.md)

### Fixed-Lag Solving - Hexagonal2D Revisited

[Hexagonal Fixed-Lag](interm_fixedlag_hexagonal.md)

### A Underdetermined Solution (unforced multimodality)
This tutorial describes a range-only system where there are always more variable dimensions than range measurements made.
The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point -- other examples show inference results where highly non-Gaussian error distributions are used.

Multi-modal range only example ([click here or image for full Vimeo](http://vimeo.com/190052649)):   
```@raw html
<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="640" border="0" /></a>
```

[Multi-Modal Under-Constrained Example](basic_slamedonut.md)

### Towards Real-Time Underwater Acoustic Navigation

This example uses "dead reckon tethering" (DRT) to perform many of the common robot odometry and high frequency pose updated operations.  These features are a staple and standard part of the distributed factor graph system.

Click on image ([or this link to Vimeo](http://vimeo.com/396532767)) for a video illustration:
```@raw html
<a href="http://vimeo.com/396532767" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/76251581-24ea0680-623f-11ea-9906-ecbe5d8ef790.gif" alt="AUV SLAM" width="640" border="0" /></a>
```

### Uncertain Data Associations, a Multi-Modal Solution (forced multi-hypothesis)

> Documentation in progress, in the mean time please see the `addFactor!(..., multihypo=[1.0; 0.5;0.5])` feature for 50/50 uncertainty. Similarly for trinary or higher multi-hypotheses per factor.

TODO: add example.

### Adding Factors - Simple Factor Design

Caesar can be extended with new variables and factors without changing the core code. An example of this design pattern is provided in this example.

[Defining New Variables and Factor](basic_definingfactors.md)


### Adding Factors - DynPose Factor

[Intermediate Example: Adding Dynamic Factors and Variables](interm_dynpose.md)

## Application Examples and Demos

### Multi-session Use-case

Multi-session [Turtlebot](http://www.turtlebot.com/) example of the second floor in the [Stata Center](https://en.wikipedia.org/wiki/Ray_and_Maria_Stata_Center):   
```@raw html
<img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/turtlemultisession.gif" alt="Turtlebot Multi-session animation" width="480" border="0" /></a>
```

See reference for more details:

> Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: ["SLAMinDB: Centralized graph databases for mobile robotics"](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf), IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017.

### Simulated Ambiguous SONAR in 3D

Intersection of ambiguous elevation angle from planar SONAR sensor:   

```@raw html
<a href="http://vimeo.com/198237738" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovasfm02.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```
Bi-modal belief   

```@raw html
<a href="http://vimeo.com/198872855" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovyaw90.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

### Synthetic Aperture Sonar SLAM

The full functional (approximate sum-product) inference approach can be used to natively imbed single hydrophone acoustic waveform data into highly non-Gaussian SAS factors--that implicitly perform beamforming/micro-location---for a simultaneous localization and mapping solution ([image links to video](https://www.youtube.com/watch?v=_RfXLQ67N4o)):

```@raw html
<a href="https://www.youtube.com/watch?v=_RfXLQ67N4o" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/52547229-3048d500-2d94-11e9-8a46-811316a45283.png" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

See reference for more details:

> Cheung, M., Fourie, D., Rypkema, N., Vaz Teixeira, P., Schmidt, H., and Leonard, J.: "Non-Gaussian SLAM utilizing Synthetic Aperture Sonar", Intl. Conf. On Robotics and Automation (ICRA), IEEE, Montreal, 2019.

## Probabilistic Data Association (Uncertain loop closures)

Example where the standard multihypothesis `addFactor!(.., multihypo=[1.0;0.5;0.5])` interface is used.  This is from the Kitti driving dataset.  [Video here](https://www.youtube.com/watch?v=9hEonD8KDrs).

```@raw html
<a href="https://www.youtube.com/watch?v=9hEonD8KDrs" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/53611733-9065b680-3b9d-11e9-8b0f-cb292a25fbb3.png" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

See reference for more details:
>  Doherty, K., Fourie, D., Leonard, J.: "Multimodal Semantic SLAM with Probabilistic Data Association", Intl. Conf. On Robotics and Automation (ICRA), IEEE, Montreal, 2019.

## More Examples

Please see examples folders for Caesar and RoME for more examples, with expanded documentation in the works.
