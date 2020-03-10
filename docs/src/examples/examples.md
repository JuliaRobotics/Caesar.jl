# Examples

The following examples demonstrate the conceptual operation of Caesar, highlighting specific features of the framework and its use.

### Continuous Scalar

This abstract [continuous scalar example](basic_continuousscalar.md) illustrates how [`IncrementalInference.jl`](http://www.github.com/JuliaRobotics/IncrementalInference.jl) enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information.

```@raw html
<p align="center">
<img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/plx0123.png" width="480" border="0" />
</p>
```

### Hexagonal 2D

#### Batch Mode

A simple [2D hexagonal robot trajectory example](basic_hexagonal2d.md) is expanded below using techniques developed in simultaneous localization and mapping (SLAM).

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/69353457-6cd82400-0c76-11ea-905c-8f435faa6b11.png" width="480" border="0" />
</p>
```

#### Bayes Tree Fixed-Lag Solving - Hexagonal2D Revisited

The [hexagonal fixed-lag](interm_fixedlag_hexagonal.md) example shows how tree based clique recycling can be achieved.  A further example is given in the real-world underwater example below.

### An Underdetermined Solution (a.k.a. SLAM-e-donut)

This tutorial describes (unforced multimodality) a range-only system where there are always more variable dimensions than range measurements made.
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

### Uncertain Data Associations, (forced multi-hypothesis)

This example presents a novel multimodal solution to an otherwise intractible multihypothesis SLAM problem.  This work spans the entire Victoria Park dataset, and resolves a solution over roughly 10000 variable dimensions with 2^1700 (yes to teh power 1700) theoretically possible modes.  At the time of first solution in 2016, a full batch solution took around 3 hours to compute on a very spartan early implementation.

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/76264526-fc233a80-6259-11ea-98aa-192d40f504f4.gif" width="480" border="0" />
</p>
```
See reference for more details:
>  [1.1] Fourie, D., Leonard, J., Kaess, M.: "A Nonparametric Belief Solution to the Bayes Tree" IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS), (2016).

Further documentation in progress, in the mean time please see the [`addFactor!(..., multihypo=[1.0; 0.5;0.5])`]() feature for fractional multi-hypothesis assignments.  Similarly for trinary or higher multi-hypotheses.
*Docs Under Construction...*

### Probabilistic Data Association (Uncertain loop closures)

Example where the standard multihypothesis `addFactor!(.., multihypo=[1.0;0.5;0.5])` interface is used.  This is from the Kitti driving dataset.  [Video here](https://www.youtube.com/watch?v=9hEonD8KDrs).

```@raw html
<a href="https://www.youtube.com/watch?v=9hEonD8KDrs" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/53611733-9065b680-3b9d-11e9-8b0f-cb292a25fbb3.png" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

See reference for more details:
>  Doherty, K., Fourie, D., Leonard, J.: "Multimodal Semantic SLAM with Probabilistic Data Association", Intl. Conf. On Robotics and Automation (ICRA), IEEE, Montreal, 2019.

### Synthetic Aperture Sonar SLAM

The full functional (approximate sum-product) inference approach can be used to natively imbed single hydrophone acoustic waveform data into highly non-Gaussian SAS factors--that implicitly perform beamforming/micro-location---for a simultaneous localization and mapping solution ([image links to video](https://www.youtube.com/watch?v=_RfXLQ67N4o)):

```@raw html
<a href="https://www.youtube.com/watch?v=_RfXLQ67N4o" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/52547229-3048d500-2d94-11e9-8a46-811316a45283.png" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

See reference for more details:

> Cheung, M., Fourie, D., Rypkema, N., Vaz Teixeira, P., Schmidt, H., and Leonard, J.: "Non-Gaussian SLAM utilizing Synthetic Aperture Sonar", Intl. Conf. On Robotics and Automation (ICRA), IEEE, Montreal, 2019.

### Simulated Ambiguous SONAR in 3D

Intersection of ambiguous elevation angle from planar SONAR sensor:   

```@raw html
<a href="http://vimeo.com/198237738" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovasfm02.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```
Bi-modal belief   

```@raw html
<a href="http://vimeo.com/198872855" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovyaw90.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

### Multi-session Indoor Robot

Multi-session [Turtlebot](http://www.turtlebot.com/) example of the second floor in the [Stata Center](https://en.wikipedia.org/wiki/Ray_and_Maria_Stata_Center):
```@raw html
<img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/turtlemultisession.gif" alt="Turtlebot Multi-session animation" width="480" border="0" /></a>
```

See [the multisession information page](https://juliarobotics.org/Caesar.jl/latest/concepts/multisession/) for more details, as well as academic work:

> Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: ["SLAMinDB: Centralized graph databases for mobile robotics"](http://people.csail.mit.edu/spillai/projects/cloud-graphs/2017-icra-cloudgraphs.pdf), IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017.

## Coming Soon

- New marine surface vehicle code tutorial using ROS.


## More Examples

Please see examples folders for Caesar and RoME for more examples, with expanded documentation in the works.

### Adding Factors - Simple Factor Design

Caesar can be extended with new variables and factors without changing the core code. An example of this design pattern is provided in this example.

[Defining New Variables and Factor](basic_definingfactors.md)


### Adding Factors - DynPose Factor

[Intermediate Example: Adding Dynamic Factors and Variables](interm_dynpose.md)
