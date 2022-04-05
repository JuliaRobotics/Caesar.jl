# On-Manifold Operations

Caesar.jl and libraries have adopted the [JuliaManifolds/Manifolds.jl](https://github.com/JuliaManifolds/Manifolds.jl) as foundation for developing the algebraic operations used.  

The Community has been developing high quality [documentation for Manifolds.jl](https://juliamanifolds.github.io/Manifolds.jl/stable/), and we encourage the interested reader to learn and use everything available there.

## Separate Manifold Beliefs Page

Follow these hyperlinks if you are looking for information on working with Manifold Beliefs or [`ManifoldKernelDensity`s](@ref manikde_page)

## Why Manifolds.jl

There is much to be said about how and why Manifolds.jl is the right decision for building a next-gen factor graph solver.  We believe the future will show that mathematicians are way ahead of the curve, and that adopting a manifold approach will pretty much be the only way to develop the required mathematical operations in Caesar.jl for the forseable future.

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

## Using Manifolds in Factors

The best way to show this is just dive straight into a factor that actually uses a Manifolds mechanization, and [`RoME.Pose2Pose2`](@ref) is a fairly straight forward example.  This factor gets used for rigid transforms on a 2D plane, with coordinates ``[x,y,\theta]`` as elluded to above.

### A Tutorial on Rotations

!!! note
    Work in progress, [Upstream Tutorial](https://github.com/JuliaManifolds/Manifolds.jl/pull/355/files)
### A Tutorial on 2D Rigid Transforms

!!! note
    Work in progress, [Upstream Tutorial](https://github.com/JuliaManifolds/Manifolds.jl/pull/366/files)

### Existing Manifolds

The most popular Manifolds used in Caesar.jl related packages are:
- [`Sphere(N)`](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/sphere.html#Manifolds.Sphere) WORK IN PROGRESS.
- [`TranslationGroup(N)`](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/group.html#Manifolds.TranslationGroup) (future work will relax to `Euclidean(N)`).
- [`SpecialOrthogonal(N)`](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/group.html#Manifolds.SpecialOrthogonal).
- [`SpecialEuclidean(N)`](https://juliamanifolds.github.io/Manifolds.jl/stable/manifolds/group.html#Special-Euclidean-group).
- `_CircleEuclid` LEGACY, TODO.
- `AMP.SE2_E2` LEGACY, TODO.

!!! note
    Caesar.jl encourages the JuliaManifolds approach to defining new manifolds, and can readily be used for Caesar.jl related operations.
    
### Creating a new Manifold

JuliaManifolds.jl is designed to make [it as easy as possible to define your own manifold and then get all the benefits of the Manifolds.jl ecosystem](https://juliamanifolds.github.io/Manifolds.jl/stable/examples/manifold.html).  Follow the documentation there to make your own manifold, which can then readily be used with all the features of both JuliaManifolds as well as the Caesar.jl related packages.

## [Answers to 7 Questions](@id seven_mani_answers)

### Q1) What are Point, Tangents, Coordinates

A manifold ``M`` is a collection of points that together create the given space.  **Points** are like round sprinkles on the donut.  The representation of points will vary from manifold to manifold.  Sometimes it is even possible to have different representations for the same point on a manifold.  These are usually denoted as ``p``.
Tangent **vectors** (we prefer _tangents_ for clarity) is a vector ``x`` that emminates from a point on a manifold.  A vector lives in the tangent space of the manifold, a local flat region around a point ``x \in T_M(p)``.  On the donut, imagine a rod-shaped sprinkle stuck along the tangent of the surface at a particular point ``p``.  The **tangent space** is the collection of all possible tangents at ``p``.  

**Coordinates** are a user defined property that uses the Euclidean nature of the tangent space at point ``p`` to operate as a regular linear space.  Coordinates are just a list of the indepedent coordinate dimensions of the tangent space values collected together.  Read this part carefully, as it can easily be confused with a conventional tangent vector in a regular Euclidean space.  

For example, a tangent vector to the ``Euclidean(2)`` manifold, at the origin point ``(0,0)`` is what you likely are familiar with from school as a "vector" (not the coordinates, although that happens to be the same thing in the trivial case).  For Euclidean space, a vector from point ``p`` of length ``[x,y]`` looks like the line segment between points ``p`` and ``q`` on the underlying manifold.  

This trivial overlapping of "vectors" in the Euclidean Manifold, and in a tangent space around ``p``, and coordinates for that tangent space, are no longer trivial when the manifold has curvature.

### Q2) What is the Logarithm map

Multiple ``x = logmap(M,p,q)`` types can exist for some manifolds.  The logarithm computes, based at point ``p``, the tangent vector ``x`` on the tangent plane ``T_M(p)`` from ``p``.  In other words, image a string following the curve of a manifold from ``p`` to ``q``, pick up that string from ``q`` while holding ``p`` firm, until the string is flat against the tangent space emminating from ``p``.  The logarithm is the opposite of the exponential map.

### Q3) What is the Exponential map

The exponential map does the opposite of the logarithm.  Image a tangent vector ``x`` emminating from point ``p``.  The length and direction of ``x`` can be wrapped onto the curvature of the manifold to form a line on the manifold surface.
### Q4) What does `vee`/`hat` do

`vee` is an operation that converts a tangent vector representation into a coordinate representation.  For example Lie algebra elements are tangent vector elements, so ``vee([0 -w; w 0]) = w``.  And visa versa for ``hat(w) = [0 -w; w 0]``, which goes from coordinates to tangent vectors.
### Q5) What Riemannian vs. Group Manifolds

Groups are defined mathematical structures which often fit well inside the manifold way of working.  For example in robotics, Lie Groups are popular under `SpecialEuclidean(N) <: AbstractGroupManifold`.  [Groups](https://en.wikipedia.org/wiki/Group_(mathematics)) also have a well defined action.  Most prominently for our usage, groups are sets of points for which there exists an identity point. [Riemannian manifolds](https://en.wikipedia.org/wiki/Riemannian_manifold) are more general that groups, specifically Riemannian manifolds do not have an identity point.  

An easy example is that the `Euclidean(N)` manifold does not have an identity element, since what we know as ``[0,0]`` is actually a coordinate base point for the local tangent space, and which just happens to look the same as the underlying `Euclidean(N)` manifold.  The `TranslationGroup(N)` exists in the `Euclidean(N)` space, but has a defined identity element as well as a defined operations on points.

### Q6) Retraction vs. Exp map

Retractions are numerically efficient approximations to convert a tangent vector onto the manifold.  The exponential map is the theoretically precise retraction, but may well be computationally expensive beyond the need for most applications.

### Q7) Projection vs. Log map

Projections are somewhat ambiguous, since on the one hand it might mean that a point from the manifold is projected up onto a nearby tangent space.  

Confusion,  however, is never too far away since there is a second type of projection that people sometimes use:  that is when a vector in the ambient space around a manifold (if one exists) is projected down onto a nearby tangent space.

There is of course also the confusion on whether a vector is projected up onto the tangent space from the manifold, or whether a tangent vector is projected down onto the manifold... It's an overloaded term, so best is to make sure you know which one is being used in any particular situation.
