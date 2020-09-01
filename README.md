<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png" width="480" border="0" />
</p>

A multimodal/non-Gaussian robotic toolkit for localization and mapping -- reducing the barrier of entry for sensor/data fusion tasks, including Simultaneous Localization and Mapping (SLAM).

# Weblink Info

Click on the badges to follow web url links:

| Stable | Dev | Documentation | Public Slack |
|:------:|:----------------:|:-------------:|:-----:|
| [![Build Status][build-tag]][build-url] | [![Build Status][build-img]][build-url] | [![docs][docs-shield]][caesar-docs] | [![][caesar-slack-badge]][caesar-slack] |


# Bleeding-edge Development Status

Code changes are currently tracked via Github's integrated Milestone/Issues/PR system -- click on the badges below to follow links into hightlights of the ecosystem.

| **Major Dependencies**  |  **Stable**     |  **Dev (master)**   |  **Test Coverage**  |  **Changes**  |
|:-----------------------:|:---------------:|:-------------------:|:-------------------:|:----------------:|
| Caesar.jl | [![][caesar-stable]][caesar-releases] | [![Build Status][build-img]][build-url] | [![codecov.io][cov-img]][cov-url] | [![][mst-shield]][caesar-milestones] |
| [RoME.jl][rome-url] | [![][r-stable]][r-releases] | [![Build Status][r-build-img]][r-build-url] | [![codecov.io][r-cov-img]][r-cov-url] | [![][mst-shield]][r-milestones] |
| [IncrementalInference.jl][iif-url] | [![][iif-stable]][iif-releases] | [![Build Status][iif-build-img]][iif-build-url] | [![codecov.io][iif-cov-img]][iif-cov-url] | [![][mst-shield]][iif-milestones] |
| [ApproxManifoldProducts.jl][amp-url] | [![][amp-stable]][amp-releases] | [![Build Status][amp-build-img]][amp-build-url] | [![codecov.io][amp-cov-img]][amp-cov-url] | [![][mst-shield]][amp-milestones] |
| [KernelDensityEstimate.jl][kde-url] | [![][kde-stable]][kde-releases] | [![Build Status][kde-build-img]][kde-build-url] | [![codecov.io][kde-cov-img]][kde-cov-url] | [![][mst-shield]][kde-milestones] |
| [FunctionalStateMachine.jl][fsm-url] | [![][fsm-stable]][fsm-releases] | [![Build Status][fsm-build-img]][fsm-build-url] | [![codecov.io][fsm-cov-img]][fsm-cov-url] | [![][mst-shield]][fsm-milestones] |
| [DistributedFactorGraphs.jl][dfg-url] | [![][dfg-stable]][dfg-releases] | [![Build Status][dfg-build-img]][dfg-build-url] | [![codecov.io][dfg-cov-img]][dfg-cov-url] | [![][mst-shield]][dfg-milestones] |
| [RoMEPlotting.jl][rp-url] | [![][rp-stable]][rp-releases] | [![Build Status][rp-build-img]][rp-build-url] | [![codecov.io][rp-cov-img]][rp-cov-url] | [![][mst-shield]][rp-milestones] |
| [TransformUtils.jl][tf-url] | [![][tf-stable]][tf-releases] | [![Build Status][tf-build-img]][tf-build-url] | [![codecov.io][tf-cov-img]][tf-cov-url] | -- |
| [LightGraphs.jl][lgraphs-url] | [![][lgjl-stable]][lgjl-releases] | [![Build Status][lgraphs-build-img]][lgraphs-build-url] | [![codecov.io][lgraphs-cov-img]][lgraphs-cov-url] | n/a |
| [~~Graphs.jl~~][graphs-url] | [![][gjl-stable]][gjl-releases] | [![Build Status][graphs-build-img]][graphs-build-url] | [![codecov.io][graphs-cov-img]][graphs-cov-url] | n/a |
| [~~CloudGraphs.jl~~][cloudgraphs-url] | ![][cg-stable] | [![Build Status][cloudgraphs-build-img]][cloudgraphs-build-url] | [![codecov.io][cloudgraphs-cov-img]][cloudgraphs-cov-url] | [![][mst-shield2]][dfg-milestones] |

# Contributors

We are grateful for many, many contributions within the Julia package ecosystem -- see the [`Project.toml`](https://github.com/JuliaRobotics/Caesar.jl/blob/master/Project.toml) files for a far reaching list of upstream packages and contributions.

Consider citing our work:

```
@misc{caesarjl,
  author = "Contributors and Dependencies",
  title =  "Caesar.jl",
  year =   2020,
  url =    "https://github.com/JuliaRobotics/Caesar.jl"
}
```

Administration of the Caesar/RoME/IncrementalInference/Arena packages is currently done by @dehann who can be contacted for more details.

# Get Involved and Code of Conduct

This project adheres to the [JuliaRobotics code of conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md), and invites contributions or comments from the community.  Use the slack channel, Julia Discourse, or Github issues to get in touch.

## Stargazers over time

[![Stargazers over time](https://starchart.cc/JuliaRobotics/Caesar.jl.svg)](https://starchart.cc/JuliaRobotics/Caesar.jl)


[docs-shield]: https://img.shields.io/badge/docs-latest-blue.svg
[caesar-docs]: http://juliarobotics.github.io/Caesar.jl/latest/
[mst-shield]: https://img.shields.io/badge/-milestones-blueviolet
[mst-shield2]: https://img.shields.io/badge/-milestones-lightgrey

[cov-img]: https://codecov.io/github/JuliaRobotics/Caesar.jl/coverage.svg?branch=master
[cov-url]: https://codecov.io/github/JuliaRobotics/Caesar.jl?branch=master
[build-img]: https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=master
[build-tag]: https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=release/v0.5
[build-url]: https://travis-ci.org/JuliaRobotics/Caesar.jl
[caesar-stable]: https://img.shields.io/badge/2020Q3-v0.6.x-green.svg
[caesar-slack-badge]: https://img.shields.io/badge/Caesarjl-Slack-green.svg?style=popout
[caesar-slack]: https://caesarjl.slack.com
[caesar-milestones]: https://github.com/JuliaRobotics/Caesar.jl/milestones
[caesar-releases]: https://github.com/JuliaRobotics/Caesar.jl/releases

[rome-url]: http://www.github.com/JuliaRobotics/RoME.jl
[r-cov-img]: https://codecov.io/github/JuliaRobotics/RoME.jl/coverage.svg?branch=master
[r-cov-url]: https://codecov.io/github/JuliaRobotics/RoME.jl?branch=master
[r-build-img]: https://travis-ci.org/JuliaRobotics/RoME.jl.svg?branch=master
[r-build-v05]: https://travis-ci.org/JuliaRobotics/RoME.jl.svg?branch=release%2Fv0.5
[r-build-url]: https://travis-ci.org/JuliaRobotics/RoME.jl
[r-stable]: https://img.shields.io/badge/2020Q3-v0.9.x-green.svg
[r-milestones]: https://github.com/JuliaRobotics/RoME.jl/milestones
[r-releases]: https://github.com/JuliaRobotics/RoME.jl/releases

[iif-cov-img]: https://codecov.io/github/JuliaRobotics/IncrementalInference.jl/coverage.svg?branch=master
[iif-cov-url]: https://codecov.io/github/JuliaRobotics/IncrementalInference.jl?branch=master
[iif-build-img]: https://travis-ci.org/JuliaRobotics/IncrementalInference.jl.svg?branch=master
[iif-build-v08]: https://travis-ci.org/JuliaRobotics/IncrementalInference.jl.svg?branch=release/v0.8
[iif-build-url]: https://travis-ci.org/JuliaRobotics/IncrementalInference.jl
[iif-url]: http://www.github.com/JuliaRobotics/IncrementalInference.jl
[iif-stable]: https://img.shields.io/badge/2020Q3-v0.15.x-green.svg
[iif-milestones]: https://github.com/JuliaRobotics/IncrementalInference.jl/milestones
[iif-releases]: https://github.com/JuliaRobotics/IncrementalInference.jl/releases

[kde-cov-img]: https://codecov.io/github/JuliaRobotics/KernelDensityEstimate.jl/coverage.svg?branch=master
[kde-cov-url]: https://codecov.io/github/JuliaRobotics/KernelDensityEstimate.jl?branch=master
[kde-build-img]: https://travis-ci.org/JuliaRobotics/KernelDensityEstimate.jl.svg?branch=master
[kde-build-url]: https://travis-ci.org/JuliaRobotics/KernelDensityEstimate.jl
[kde-url]: http://www.github.com/JuliaRobotics/KernelDensityEstimate.jl
[kde-stable]: https://img.shields.io/badge/2019Q1-v0.5.x-green.svg
[kde-milestones]: https://github.com/JuliaRobotics/KernelDensityEstimate.jl/milestones
[kde-releases]: https://github.com/JuliaRobotics/KernelDensityEstimate.jl/releases

[tf-cov-img]: https://codecov.io/github/dehann/TransformUtils.jl/coverage.svg?branch=master
[tf-cov-url]: https://codecov.io/github/dehann/TransformUtils.jl?branch=master
[tf-build-img]: https://travis-ci.org/dehann/TransformUtils.jl.svg?branch=master
[tf-build-url]: https://travis-ci.org/dehann/TransformUtils.jl
[tf-url]: http://www.github.com/dehann/TransformUtils.jl
[tf-stable]: https://img.shields.io/badge/2018Q4-v0.2.x-green.svg
[tf-milestones]: https://github.com/JuliaRobotics/TransformUtils.jl/milestones
[tf-releases]: https://github.com/JuliaRobotics/TransformUtils.jl/releases

<!-- | [DrakeVisualizer.jl][dvis-url] | [![Build Status][dvis-build-img]][dvis-build-url] | [![codecov.io][dvis-cov-img]][dvis-cov-url] |
[dvis-cov-img]: https://codecov.io/github/rdeits/DrakeVisualizer.jl/coverage.svg?branch=master
[dvis-cov-url]: https://codecov.io/github/rdeits/DrakeVisualizer.jl?branch=master
[dvis-build-img]: https://travis-ci.org/rdeits/DrakeVisualizer.jl.svg?branch=master
[dvis-build-url]: https://travis-ci.org/rdeits/DrakeVisualizer.jl
[dvis-url]: http://www.github.com/rdeits/DrakeVisualizer.jl -->

[graphs-cov-img]: https://codecov.io/github/JuliaAttic/Graphs.jl/coverage.svg?branch=master
[graphs-cov-url]: https://codecov.io/github/JuliaAttic/Graphs.jl?branch=master
[graphs-build-img]: https://travis-ci.org/JuliaAttic/Graphs.jl.svg?branch=master
[graphs-build-url]: https://travis-ci.org/JuliaAttic/Graphs.jl
[graphs-url]: http://www.github.com/JuliaAttic/Graphs.jl
[gjl-stable]: https://img.shields.io/badge/2019Q2-v0.10.x-green.svg
[gjl-releases]: https://github.com/JuliaAttic/Graphs.jl/releases

[lgraphs-cov-img]: https://codecov.io/github/JuliaGraphs/LightGraphs.jl/coverage.svg?branch=master
[lgraphs-cov-url]: https://codecov.io/github/JuliaGraphs/LightGraphs.jl?branch=master
[lgraphs-build-img]: https://travis-ci.org/JuliaGraphs/LightGraphs.jl.svg?branch=master
[lgraphs-build-url]: https://travis-ci.org/JuliaGraphs/LightGraphs.jl
[lgraphs-url]: http://www.github.com/JuliaGraphs/LightGraphs.jl
[lgjl-stable]: https://img.shields.io/badge/2019Q3-v1.3.x-green.svg
[lgjl-releases]: https://github.com/JuliaGraphs/LightGraphs.jl/releases

[dfg-cov-img]: https://codecov.io/github/JuliaRobotics/DistributedFactorGraphs.jl/coverage.svg?branch=master
[dfg-cov-url]: https://codecov.io/github/JuliaRobotics/DistributedFactorGraphs.jl?branch=master
[dfg-build-img]: https://travis-ci.org/JuliaRobotics/DistributedFactorGraphs.jl.svg?branch=master
[dfg-build-url]: https://travis-ci.org/JuliaRobotics/DistributedFactorGraphs.jl
[dfg-url]: http://www.github.com/JuliaRobotics/DistributedFactorGraphs.jl
[dfg-stable]: https://img.shields.io/badge/2020Q3-v0.10.x-green.svg
[dfg-milestones]: https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/milestones
[dfg-releases]: https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/releases

[amp-cov-img]: https://codecov.io/github/JuliaRobotics/ApproxManifoldProducts.jl/coverage.svg?branch=master
[amp-cov-url]: https://codecov.io/github/JuliaRobotics/ApproxManifoldProducts.jl?branch=master
[amp-build-img]: https://travis-ci.org/JuliaRobotics/ApproxManifoldProducts.jl.svg?branch=master
[amp-build-url]: https://travis-ci.org/JuliaRobotics/ApproxManifoldProducts.jl
[amp-url]: http://www.github.com/JuliaRobotics/ApproxManifoldProducts.jl
[amp-stable]: https://img.shields.io/badge/2020Q2-v0.1.x-green.svg
[amp-milestones]: https://github.com/JuliaRobotics/ApproxManifoldProducts.jl/milestones
[amp-releases]: https://github.com/JuliaRobotics/ApproxManifoldProducts.jl/releases

[fsm-cov-img]: https://codecov.io/github/JuliaRobotics/FunctionalStateMachine.jl/coverage.svg?branch=master
[fsm-cov-url]: https://codecov.io/github/JuliaRobotics/FunctionalStateMachine.jl?branch=master
[fsm-build-img]: https://travis-ci.org/JuliaRobotics/FunctionalStateMachine.jl.svg?branch=master
[fsm-build-url]: https://travis-ci.org/JuliaRobotics/FunctionalStateMachine.jl
[fsm-url]: http://www.github.com/JuliaRobotics/FunctionalStateMachine.jl
[fsm-stable]: https://img.shields.io/badge/2020Q3-v0.2.x-green.svg
[fsm-milestones]: https://github.com/JuliaRobotics/FunctionalStateMachine.jl/milestones
[fsm-releases]: https://github.com/JuliaRobotics/FunctionalStateMachine.jl/releases

[cloudgraphs-cov-img]: https://codecov.io/github/GearsAD/CloudGraphs.jl/coverage.svg?branch=master
[cloudgraphs-cov-url]: https://codecov.io/github/GearsAD/CloudGraphs.jl?branch=master
[cloudgraphs-build-img]: https://travis-ci.org/GearsAD/CloudGraphs.jl.svg?branch=master
[cloudgraphs-build-url]: https://travis-ci.org/GearsAD/CloudGraphs.jl
[cloudgraphs-url]: http://www.github.com/GearsAD/CloudGraphs.jl
[cg-stable]: https://img.shields.io/badge/2019Q3-WIP-yellowgreen.svg

[rp-url]: http://www.github.com/JuliaRobotics/RoMEPlotting.jl
[rp-cov-img]: https://codecov.io/github/JuliaRobotics/RoMEPlotting.jl/coverage.svg?branch=master
[rp-cov-url]: https://codecov.io/github/JuliaRobotics/RoMEPlotting.jl?branch=master
[rp-build-img]: https://travis-ci.org/JuliaRobotics/RoMEPlotting.jl.svg?branch=master
[rp-build-url]: https://travis-ci.org/JuliaRobotics/RoMEPlotting.jl
[rp-stable]: https://img.shields.io/badge/2020Q3-v0.4.x-green.svg
[rp-milestones]: https://github.com/JuliaRobotics/RoMEPlotting.jl/milestones
[rp-releases]: https://github.com/JuliaRobotics/RoMEPlotting.jl/releases
