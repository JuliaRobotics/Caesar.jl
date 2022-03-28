# On-Manifold Operations

Caesar.jl and libraries has adopted the [JuliaManifolds/Manifolds.jl](https://github.com/JuliaManifolds/Manifolds.jl) as foundation for developing the algebraic operations used.  

The Community has been developing high quality [documentation for Manifolds.jl](https://juliamanifolds.github.io/Manifolds.jl/stable/), and we encourage the interested reader to learn and use everything available there.

## Separate Manifold Beliefs Page

Follow these hyperlinks if you are looking for information on working with Manifold Beliefs or [`ManifoldKernelDensity`s](@ref)

## Why Manifolds.jl

There is much to be said about how and why Manifolds.jl is the right decision for building a factor graph solver ecosystems.  We believe the future will show that mathematicians are way ahead of the curve, and that adopting a manifold approach will pretty much be the only way to develop the required mathematical operations in Caesar.jl for the forseable future.

### Are Manifolds Difficult? No.

Do you need a math degree to be able to use Manifolds.jl?  No you don't since Caesar.jl and related packages have already packaged many of the common functions and factors you need to get going.  

This page is meant to open the door for readers to learn more about how things work under the hood, and empower the Community to rapidly develop upon  existing work.  This page is also intended to show that the Caesar.jl related packages are being developed with strong focus on consolidation, single definition functionality, and serious cross discipline considerations.

If you are looking for rapid help or more expertise on a particular issue, consider reaching out by opening [Issues](https://github.com/JuliaRobotics/Caesar.jl/issues) or connecting to the ongoing chats in the [Slack Channel](https://join.slack.com/t/caesarjl/shared_invite/zt-ucs06bwg-y2tEbddwX1vR18MASnOLsw).

## What Are Manifolds

If you are a newcomer to the term Manifold and want to learn more, fear not even though your first search results might be somewhat disorienting.  

The rest of this page is meant to introduce the basics, and point you to handy resources.  Caesar.jl and [NavAbility](https://www.navability.io/) support open Community and are upstreaming improvements to Manifolds.jl, including code updates and documentation improvements.

### 'One Page' Summary of Manifolds

Imagine you have a sheet of paper and you draw with a pencil a short line segment on the page.  Now draw a second line segment from the end of the first.  That should be pretty easy on a flat surface, right?

When the piece of paper is lying flat on the table, you have a line in the [`Euclidean(2)` manifold](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/euclidean.html), and you can easily assign `[x,y]` coordinates to describe these lines or vectors.  **Note** _coordinates_ here is a precise technical term.

If you roll the paper into a cyclinder... well now you have line segments on a cylindrical manifold.  The question is, how to conduct mathematical operations concisely and consistently indepependent of the shape of your manifold?  And, how to 'unroll' the paper for simple computations on a locally flat surface.

How far can the math go before there just isn't a good recipe for writing down generic operations?  Turns out a few smart people have been working to solve this and the keyword here is _Manifold_.

If you are drinking some coffee right now, then you are moving the cup in `Euclidean(3)` space, that is you assume the motion is in flat **coordinates** `[x;y;z]`.  A more technical way to say that is that the `Euclidean` manifold has zero [_curvature_](https://en.wikipedia.org/wiki/Curvature).  

What if you are concerned with the orientation of the cup too---as in not spill the hot contents everywhere---then you might actually want to work on the [`SpecialEuclidean(3)`](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/group.html#Special-Euclidean-group) manifold -- that is 3 degrees of translational freedom, and 3 degrees of rotational freedom.  You might have heard of [Lie Groups](https://en.wikipedia.org/wiki/Lie_group) and [Lie Algebras](https://en.wikipedia.org/wiki/Lie_algebra), well that is exactly it, Lie Groups are a special set of Group Manifolds and associated operations that are already supported by [JuliaManifolds/Manifolds.jl](https://github.com/JuliaManifolds/Manifolds.jl).

Things are a little easier for a robot traveling around on a flat 2D surface.  If you robot is moving around with coordinates ``[x,y,\theta]``, well then you are working with the coordinates of the `SpecialEuclidean(2)` manifold.  There is more to say on how you the coordinates ``[x,y,\theta]`` get converted into the `se(2)` Lie algebra, and that gets converted into a Lie Group element -- i.e. ``([x;y], RotMat(\theta))``.  More on that later.

Perhaps you are interested in relativistic effects where time as the fourth dimension is of interest, well then the [Minkowski space](https://en.wikipedia.org/wiki/Minkowski_space) provides Group and Manifold constructs for that -- actually Minkowski falls under the supported [Lorentz Manifolds](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/lorentz.html).

The point here is that the math for drawing line segments in each of these manifolds above is almost exactly the same, thanks to the abstractions that have already been developed.  And, many more powerful constructs exist which will become more apparent as you continue to work with Manifolds.

### 7 Things to know First

As a robotics, navigation, or control person who wants to get started, you need to know what the following terms mean:
- Q1) What are manifold _points_, tangent _vectors_, and user _coordinates_,
- Q2) What does the logarithm map of a manifold do,
- Q3) What does the exponential map of a manifold do,
- Q4) What do the `vee` and `hat` operations do,
- Q5) What is the difference between Riemannian or Group manifolds,
- Q6) Is a retraction the same as the exponential map,
- Q7) Is a projection the same as a logarithm map,

Know it sounds like a lot, but the point of this paragraph is that if you are able to answer these seven questions for yourself, then you will be empowered to venture into the math of manifolds much more easily.  And, everything will begin to make sense.  A lot of sense, to the point that you might agree with our assesment that JuliaManifolds/Manifolds.jl is the right direction for the future.

Although you will be able to find many answers for these seven questions in many places, our [answers are listed at the bottom of this page](@ref seven_mani_answers).

### Manifold Tutorials

The rest of this page is devoted to showing you how to use the math, write your own code to do new things beyond what Caesar.jl can already do.  If are willing to share any contributions, please do so by opening pull requests against the related repos.

### A Tutorial on Rotations


### A Tutorial on 2D Rigid Transforms


## How to use in Factors


### Existing Manifolds


### Creating a new Manifold


## [Answers to 7 Questions](@id seven_mani_answers)

### Q1) What are Point, Tangents, Coordinates

### Q2) What is the Logarithm map

### Q3) What is the Exponential map

### Q4) What does `vee`/`hat` do

### Q5) What Riemannian vs. Group Manifolds

### Q6) Retraction vs. Exp map

### Q7) Projection vs. Log map