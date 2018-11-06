# Examples

## Basics
The following examples demonstrate the conceptual operation of Caesar, highlighting specific features of the framework and its use.

### Hexagonal 2D
A simple 2D robot trajectory example is expanded below using techniques developed in simultaneous localization and mapping (SLAM).

[Hexagonal 2D Example](basic_hexagonal2d.md)

### A Multi-Modal Under-Constrained Solution
This tutorial describes a range-only system where there are always more variable dimensions than range measurements made.
The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point -- other examples show inference results where highly non-Gaussian error distributions are used.

Multi-modal range only example ([click here or image for full Vimeo](http://vimeo.com/190052649)):   
```@raw html
<a href="http://vimeo.com/190052649" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif" alt="IMAGE ALT TEXT HERE" width="640" border="0" /></a>
```

[Multi-Modal Under-Constrained Example](basic_slamedonut.md)

### Continuous Scalar

This abstract example illustrates how `IncrementalInference` enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information.

[Continuous Scalar Example](basic_continuousscalar.md)

### Adding Factors - Simple Factor Design

Caesar can be extended with new variables and factors without changing the core code. An example of this design pattern is provided in this example.

[Defining New Variables and Factor](basic_definingfactors.md)

## Intermediate Examples
The following are more complex examples that demonstrate real-world applications.

### Fixed-Lag Solving - Hexagonal2D Revisited

[Hexagonal Fixed-Lag](interm_fixedlag_hexagonal.md)

### Adding Factors - DynPose Factor

[Intermediate Example: Adding Dynamic Factors and Variables](interm_dynpose.md)

## Application Examples and Demos

### Multi-session Use-case

Multi-session [Turtlebot](http://www.turtlebot.com/) example of the second floor in the [Stata Center](https://en.wikipedia.org/wiki/Ray_and_Maria_Stata_Center):   
```@raw html
<img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/turtlemultisession.gif" alt="Turtlebot Multi-session animation" width="480" border="0" /></a>
```

### Simulated Ambiguous SONAR in 3D

Intersection of ambiguous elevation angle from planar SONAR sensor:   

```@raw html
<a href="http://vimeo.com/198237738" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovasfm02.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```
Bi-modal belief   

```@raw html
<a href="http://vimeo.com/198872855" target="_blank"><img src="https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovyaw90.gif" alt="IMAGE ALT TEXT HERE" width="480" border="0" /></a>
```

## More Examples

Please see examples folders for Caesar and RoME for more examples, with expanded documentation in the works.
