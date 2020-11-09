# Dead Reckon Tether

Towards real-rime location prediction and model based target tracking.  See brief description in this presentation.

```@raw html
<iframe src="https://player.vimeo.com/video/474897929#t=11m24s" width="800" frameborder="0" allow="autoplay; fullscreen" allowfullscreen></iframe>
<p><a href="https://vimeo.com/474897929">Towards Real-Time Non-Gaussian SLAM</a> from <a href="https://vimeo.com/user35117400">Dehann</a> on <a href="https://vimeo.com">Vimeo</a>.</p>
```
    
## Functions to Use

See the related functions while this documentation is being expanded:

- `addVariable!(fg, :drt_0, ..., solvable=0)`
- `drec1 = MutablePose2Pose2Gaussian(...)`
- `addFactor!(dfg, [:x0; :drt_0], drec1, solvable=0, graphinit=false)`
- `accumulateDiscreteLocalFrame!`
- `accumulateFactorMeans`
- `duplicateToStandardFactorVariable`

