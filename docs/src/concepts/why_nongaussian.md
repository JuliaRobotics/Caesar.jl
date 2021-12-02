# [Why/Where does non-Gaussian data come from?](@id why_nongaussian)

Gaussian error models in measurement or data cues will only be Gaussian (normally distributed) if all physics/decisions/systematic-errors/calibration/etc. has a correct algebraic model in all circumstances.  Caesar.jl and MM-iSAMv2 is heavily focussed on state-estimation from a plethora of heterogenous data that may not yet have perfect algebraic models.  Four major categories of non-Gaussian errors have thus far been considered:
- Uncertain decisions (a.k.a. data association), such as a robot trying to decide if a navigation loop-closure can be deduced from a repeat observation of a similar object or measurement from current and past data.  These issues are commonly also referred to as multi-hypothesis.
- Underdetermined or underdefined systems where there are more variables than constraining measurements to fully define the system as a single mode---a.k.a solution ambiguity.  For example, in 2D consider two range measurements resulting in two possible locations through trilateration.
- Nonlinearity.  For example in 2D, consider a Pose2 odometry where the orientation is uncertain:  The resulting belief of where a next pose might be (convolution with odometry factor) results in a banana shape curve, even though the entire process is driven by assumed Gaussian belief.
- Physics of the measurement process.  Many measurement processes exhibit non-Gaussian behaviour.  For example, acoustic/radio time-of-flight measurements, using either pulse-train or matched filtering, result in an "energy intensity" over time/distance of what the range to a scattering-target/source might be--i.e. highly non-Gaussian.

## Next Steps

Quick links to related pages:

```@contents
Pages = [
    "installation_environment.md"
    "concepts/concepts.md"
    "concepts/building_graphs.md"
    "concepts/2d_plotting.md"
]
Depth = 1
```
