# Examples

The following examples demonstrate the conceptual operation of Caesar, highlighting specific features of the framework and its use.

## Continuous Scalar

### Calculating a Square Root (Underdetermined)

Probably the most minimal example that illustrates how factor graphs represent a mathematical framework is a reworking of the classic square root calculation.

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/98160784-b2542800-1eac-11eb-81ca-4997a8a7b25c.png" width="480" border="0" />
</p>
```

!!! note
    WIP, a combined type-definion and square root script is [available as an example script](https://github.com/JuliaRobotics/IncrementalInference.jl/blob/master/examples/SquareRootTypes.jl).  We're working to present the example without having to define any types.

### Continuous Scalar with Mixtures

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

This tutorial describes (unforced multimodality) a range-only system where there are always more variable dimensions than range measurements made, see [Underdeterminied Example here](basic_slamedonut.md)
The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point -- other examples show inference results where highly non-Gaussian error distributions are used.

Multi-modal range only example ([click here or image for full Vimeo](http://vimeo.com/190052649)):   
```@raw html
<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="640" border="0" /></a>
```

### Towards Real-Time Underwater Acoustic Navigation

This example uses "dead reckon tethering" (DRT) to perform many of the common robot odometry and high frequency pose updated operations.  These features are a staple and standard part of the distributed factor graph system.

Click on image ([or this link to Vimeo](http://vimeo.com/396532767)) for a video illustration:
```@raw html
<a href="http://vimeo.com/396532767" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/76251581-24ea0680-623f-11ea-9906-ecbe5d8ef790.gif" alt="AUV SLAM" width="640" border="0" /></a>
```

### Uncertain Data Associations, (forced multi-hypothesis)

This example presents a novel multimodal solution to an otherwise intractible multihypothesis SLAM problem.  This work spans the entire Victoria Park dataset, and resolves a solution over roughly 10000 variable dimensions with 2^1700 (yes to teh power 1700) theoretically possible modes.  At the time of first solution in 2016, a full batch solution took around 3 hours to compute on a very spartan early implementation.

```@raw html
<img src="https://user-images.githubusercontent.com/6412556/76264526-fc233a80-6259-11ea-98aa-192d40f504f4.gif" width="480" border="0" />
</p>
```

The fractional multi-hypothesis assignments `addFactor!(..., multihypo=[1.0; 0.5;0.5])`.  The [Multihypothesis](@ref) Section discusses this feature in more detail.  Similarly for tri-nary or higher multi-hypotheses.

### Probabilistic Data Association (Uncertain loop closures)

Example where the standard multihypothesis `addFactor!(.., multihypo=[1.0;0.5;0.5])` interface is used.  This is from the Kitti driving dataset.  [Video here](https://www.youtube.com/watch?v=9hEonD8KDrs).  The [Multihypothesis](@ref) Section discusses this feature in more detail.

```@raw html
<a href="https://www.youtube.com/watch?v=9hEonD8KDrs" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/53611733-9065b680-3b9d-11e9-8b0f-cb292a25fbb3.png" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

### Synthetic Aperture Sonar SLAM

The full functional (approximate sum-product) inference approach can be used to natively imbed single hydrophone acoustic waveform data into highly non-Gaussian SAS factors--that implicitly perform beamforming/micro-location---for a simultaneous localization and mapping solution ([image links to video](https://www.youtube.com/watch?v=_RfXLQ67N4o)).  See the [Raw Correlator Probability (Matched Filter)](@ref) Section for more details.

```@raw html
<a href="https://www.youtube.com/watch?v=_RfXLQ67N4o" target="_blank"><img src="https://user-images.githubusercontent.com/6412556/52547229-3048d500-2d94-11e9-8a46-811316a45283.png" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

### Marine Surface Vehicle with ROS

- New marine surface vehicle code tutorial using ROS.

!!! note
    See [initial example here](https://github.com/JuliaRobotics/Caesar.jl/tree/master/examples/marine/asv/rex), and native ROS support section here.

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


## More Examples

Please see examples folders for Caesar and RoME for more examples, with expanded documentation in the works.

### Adding Factors - Simple Factor Design

Caesar can be extended with new variables and factors without changing the core code. An example of this design pattern is provided in this example.

[Defining New Variables and Factor](basic_definingfactors.md)


### Adding Factors - DynPose Factor

[Intermediate Example: Adding Dynamic Factors and Variables](../principles/interm_dynpose.md)
