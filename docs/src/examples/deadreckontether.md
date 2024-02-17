# Dead Reckon Tether

Towards real-rime location prediction and model based target tracking.  See brief description in this presentation.

```@raw html
<iframe src="https://player.vimeo.com/video/474897929#t=11m24s" width="800" frameborder="0" allow="autoplay; fullscreen" allowfullscreen></iframe>
<p><a href="https://vimeo.com/474897929">Towards Real-Time Non-Gaussian SLAM</a> from <a href="https://vimeo.com/user35117400">Dehann</a> on <a href="https://vimeo.com">Vimeo</a>.</p>
```
    
## DRT Functions  

Overview of related functions while this documentation is being expanded:

- `addVariable!(fg, :drt_0, ..., solvable=0)`
- `drec1 = MutablePose2Pose2Gaussian(...)`
- `addFactor!(dfg, [:x0; :drt_0], drec1, solvable=0, graphinit=false)`
- `accumulateDiscreteLocalFrame!`
- `accumulateFactorMeans`
- `duplicateToStandardFactorVariable`

## DRT Construct

The idea is that the dead reckong tracking method is to update a single value based on high-rate sensor data.  Perhaps 'particles' values can be propagated as a non-Gaussian prediction, depending on allowable compute resources, and for that see [`approxConvBelief`](@ref).  Some specialized plumbing has been built to facilitate rapid single value propagation using the factor graph.  

### Suppress w/ `solvable`

The construct uses regular [`addVariable!`](@ref) and [`addFactor!`](@ref) calls but with a few tweaks.  The first is that some variables and factors should not be incorporated with the regular [`solveTree!`](@ref) call and can be achieved on a per node basis, e.g.:
```julia
fg = initfg()

# a regular variable and prior for solving in graph
addVariable!(fg, :x0, Pose2) # default solvable=1
addFactor!(fg, [:x0;], PriorPose2(MvNormal([0;0;0.0],diagm([0.1;0.1;0.01]))))

# now add a variable that will not be included in solves
addVariable!(fg, :drt0, Pose2, solvable=0)
```

### A Mutable Factor

The next part is to add a factor that can be rapidly updated from sensor data, hence liberal use of the term 'Mutable':
```julia
drt0 = MutablePose2Pose2Gaussian(MvNormal([0;0;0.0],diagm([0.1;0.1;0.01])))
addFactor!(dfg, [:x0; :drt0], drt0, solvable=0, graphinit=false)
```

Notice that this factor is also set with `solvable=0` to exclude it from the regular solving process.  Also note the `graphinit=false` to prevent any immediate automated attempts to initialize the values to connected variables using this factor.

### Sensor rate updates

The idea of a dead reckon tether is that the value in the factor can rapidly be updated without affecting any other regular part of the factor graph or simultaneous solving progress.  Imagine new sensor data from wheel odometry or an IMU is available which is then used to 'mutate' the values in a DRT factor:
```julia
# continuous Gaussian process noise Q
Qc = 0.001*diagm(ones(3))

# accumulate a Pose2 delta odometry measurement segment onto existing value in drt0
accumulateDiscreteLocalFrame!(drt0,[0.1;0;0.05],Qc)
```

### Dead Reckoned Prediction

Using the latest available inference result `fg[:x0]`, the `drt0` factor can be used to predict the single parameteric location of variable `:drt0`:
```julia
# can happen concurrently with most other operations on fg, including `solveTree!`
predictDRT0 = accumulateFactorMeans(fg, [:x0drt0f1;])
```

Note also a convenience function uses similar plumbing for integrating odometry as well as any other DRT operations.  Imagine a robot is driving from pose position 0 to 1, then the final pose trigger value in factor `drt0` is the same value required to instantiate a new factor graph `Pose2Pose2`, and hence:
```julia
# add new regular rigid transform (odometry) factor between pose variables 
duplicateToStandardFactorVariable(Pose2Pose2, drt0, fg, :x0, :x1)
```

!!! warning
    (2021Q1) Some of these function names are likely to be better standardized in the future.  Regular semver deprecation warnings will be used to simplify any potential updates that may occur.  Please file issues at Caesar.jl if any problems arise.

## Function Reference

```@docs
duplicateToStandardFactorVariable
accumulateDiscreteLocalFrame!
accumulateFactorMeans
MutablePose2Pose2Gaussian
```

## Additional Notes

This will be consolidated with text above:
- regardless of slam solution going on in the background, you can then just call `val = accumulateFactorMeans(fg, [:x0deadreckon_x0f1])`
for a new dead reckon tether solution;
- you can add as many tethers as you want.  
- So if you solving every 10 poses, you just add a new tether x0, x10, x20, x30...
- as the solves complete on previous segments, then you can just get the latest `accumulateFactorMean`
