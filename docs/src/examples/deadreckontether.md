# Dead Reckon Tether

Towards real-rime location prediction and model based target tracking.

!!! note

    This page is under construction (2Q20).
    
## Functions to Use

- `addVariable!(fg, :drt_0, ..., solvable=0)`
- `drec1 = MutablePose2Pose2Gaussian(...)`
- `addFactor!(dfg, [:x0; :drt_0], drec1, solvable=0, graphinit=false)`
- `accumulateDiscreteLocalFrame!`
- `accumulateFactorMeans`
- `duplicateToStandardFactorVariable`

