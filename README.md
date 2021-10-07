<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/134777705-acda768e-884b-4de6-b307-aac6d09b9c81.png" width="480" border="0" />
</p>

A multimodal/non-Gaussian robotic toolkit for localization and mapping -- reducing the barrier of entry for sensor/data fusion tasks, including Simultaneous Localization and Mapping (SLAM).

[NavAbility.io](http://www.navability.io) helps the with administration and support of the Caesar.jl community, please reach out for any additional information (info@navability.io) or via the Slack badge-link below.

# Weblink Info

Click on the badges to follow web url links:

| Stable v0.10 | Dev | Documentation | caesarjl.slack.com |
|:------:|:----------------:|:-------------:|:-----:|
| [![Build Status][build-v0.10]][build-url] | [![Build Status][build-img]][build-url] <br> [![ColPrac][colp-badge]][colprac] | [![docs][docs-shield]][caesar-docs] | [![][caesar-slack-badge]][caesar-slack] |

Also find factor graph solver DOI at:
[![doi-img]][doi-url]

# Bleeding-edge Development Status

Code changes are currently tracked via Github's integrated Milestone/Issues/PR system -- click on the badges below to follow links into hightlights of the ecosystem.

| **Major Dependencies**  |  **Stable Release**     |  **Dev Branch**   |  **Test Coverage**  |  **Changes**  |
|:-----------------------:|:---------------:|:-------------------:|:-------------------:|:----------------:|
| Caesar.jl | [![version][cjl-stbl-img]][caesarjl-releases] | [![Build Status][build-img]][build-url] | -*umbrella pkg*- | [![][mst-shield]][caesarjl-milestones] |
| [RoME.jl][rjl-url] | [![][rjl-ver-img]][rjl-releases] | [![Build Status][rjl-ci-dev-img]][rjl-ci-dev-url] | [![codecov.io][rjl-cov-img]][rjl-cov-url] | [![][mst-shield]][rjl-milestones] |
| [IncrementalInference.jl][iif-url] | [![][iif-ver-img]][iif-rel-url] | [![Build Status][iif-ci-dev-img]][iif-ci-dev-url] | [![codecov.io][iif-cov-img]][iif-cov-url] | [![][mst-shield]][iif-milestones] |
| [ApproxManifoldProducts.jl][amp-url] | [![][amp-stable]][amp-releases] | [![Build Status][amp-build-img]][amp-build-url] | [![codecov.io][amp-cov-img]][amp-cov-url] | [![][mst-shield]][amp-milestones] |
| [KernelDensityEstimate.jl][kde-url] | [![][kde-stable]][kde-releases] | [![Build Status][kde-build-img]][kde-build-url] | [![codecov.io][kde-cov-img]][kde-cov-url] | [![][mst-shield]][kde-milestones] |
| [FunctionalStateMachine.jl][fsm-url] | [![][fsm-stable]][fsm-releases] | [![Build Status][fsm-build-img]][fsm-build-url] | [![codecov.io][fsm-cov-img]][fsm-cov-url] | [![][mst-shield]][fsm-milestones] |
| [DistributedFactorGraphs.jl][dfg-url] | [![][dfg-stable]][dfg-releases] | [![Build Status][dfg-build-img]][dfg-build-url] | [![codecov.io][dfg-cov-img]][dfg-cov-url] | [![][mst-shield]][dfg-milestones] |
| [LightGraphs.jl][lgraphs-url] | [![][lgjl-stable]][lgjl-releases] | [![Build Status][lgraphs-build-img]][lgraphs-build-url] | [![codecov.io][lgraphs-cov-img]][lgraphs-cov-url] | upstream |
| [Manifolds.jl][mani-url] | [![][mani-stable]][mani-releases] | [![Build Status][mani-build-img]][mani-build-url] | [![codecov.io][mani-cov-img]][mani-cov-url] | upstream |
| [Optim.jl][optim-url] | [![][optim-stable]][optim-releases] | [![Build Status][optim-build-img]][optim-build-url] <br> [![Build Status][optim-img-windows]][optim-build-windows] | [![codecov.io][optim-cov-img]][optim-cov-url] | upstream |
| [TransformUtils.jl][tf-url] | [![][tf-stable]][tf-releases] | [![Build Status][tf-build-img]][tf-build-url] | [![codecov.io][tf-cov-img]][tf-cov-url] | deprecating |
| [RoMEPlotting.jl][rp-url] | [![][rp-stable]][rp-releases] | [![Build Status][rp-build-img]][rp-build-url] | [![codecov.io][rp-cov-img]][rp-cov-url] | [![][mst-shield]][rp-milestones] |

# Contributors

We are grateful for many, many contributions within the Julia package ecosystem -- see the [Juliahub.com page](https://juliahub.com/ui/Packages/Caesar/BNbRm/0.10.2?t=1) for the upstream package dependencies.

Consider citing our work:
```
@online{Caesarjl2021,
 author =        {{P}ackage {C}ontributors and {E}cosystem},
 title =         {Caesar.jl, v0.10.2},
 year =          {2021},
 doi=            {Solver DOI: 10.5281/zenodo.5146222}, 
 note =          {\url{https://github.com/JuliaRobotics/Caesar.jl}}
}
```

## Get Involved and Code of Conduct

This project adheres to the [JuliaRobotics code of conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md), and invites contributions or comments from the community.  Use the Slack channel, Julia Discourse, or Github issues to get in touch.

## Stargazers over time

[![Stargazers over time](https://starchart.cc/JuliaRobotics/Caesar.jl.svg)](https://starchart.cc/JuliaRobotics/Caesar.jl)

[doi-img]: https://zenodo.org/badge/55802838.svg
[doi-url]: https://zenodo.org/badge/latestdoi/55802838

[colp-badge]: https://img.shields.io/badge/ColPrac-Contributor's%20Guide-blueviolet
[colprac]: https://github.com/SciML/ColPrac

[docs-shield]: https://img.shields.io/badge/docs-latest-blue.svg
[caesar-docs]: http://juliarobotics.github.io/Caesar.jl/latest/
[mst-shield]: https://img.shields.io/badge/-milestones-blueviolet
[mst-shield2]: https://img.shields.io/badge/-milestones-lightgrey

[cov-img]: https://codecov.io/github/JuliaRobotics/Caesar.jl/coverage.svg?branch=master
[cov-url]: https://codecov.io/github/JuliaRobotics/Caesar.jl?branch=master
[build-img]: https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=master
[build-v0.10]: https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=release/v0.10
[build-v0.9]: https://travis-ci.org/JuliaRobotics/Caesar.jl.svg?branch=release/v0.9
[build-url]: https://travis-ci.org/JuliaRobotics/Caesar.jl

[cjl-stbl-img]: https://juliahub.com/docs/Caesar/version.svg
[cjl-stbl-url]: https://juliahub.com/ui/Packages/Caesar/BNbRm
[caesar-slack-badge]: https://img.shields.io/badge/Invite-Slack-green.svg?style=popout
[caesar-slack]: https://join.slack.com/t/caesarjl/shared_invite/zt-ucs06bwg-y2tEbddwX1vR18MASnOLsw
[caesarjl-milestones]: https://github.com/JuliaRobotics/Caesar.jl/milestones
[caesarjl-releases]: https://github.com/JuliaRobotics/Caesar.jl/releases

<!-- md variables duplicated in RoME.jl README -->
[rjl-url]: http://www.github.com/JuliaRobotics/RoME.jl
[rjl-cov-img]: https://codecov.io/github/JuliaRobotics/RoME.jl/coverage.svg?branch=master
[rjl-cov-url]: https://codecov.io/github/JuliaRobotics/RoME.jl?branch=master
[rjl-ci-dev-img]: https://github.com/JuliaRobotics/RoME.jl/actions/workflows/ci.yml/badge.svg
[rjl-ci-dev-url]: https://github.com/JuliaRobotics/RoME.jl/actions/workflows/ci.yml
[rjl-ver-img]: https://juliahub.com/docs/RoME/version.svg
[rjl-milestones]: https://github.com/JuliaRobotics/RoME.jl/milestones
[rjl-releases]: https://github.com/JuliaRobotics/RoME.jl/releases
[rjl-juliahub]: https://juliahub.com/ui/Packages/RoME/VVxXB

<!-- variables duplicated in IncrementalInference.jl README -->
[iif-ci-dev-img]: https://github.com/JuliaRobotics/IncrementalInference.jl/actions/workflows/ci.yml/badge.svg
[iif-ci-dev-url]: https://github.com/JuliaRobotics/IncrementalInference.jl/actions/workflows/ci.yml
[iif-ci-stb-img]: https://github.com/JuliaRobotics/IncrementalInference.jl/actions/workflows/ci.yml/badge.svg?branch=release%2Fv0.25
[iif-ci-stb-url]: https://github.com/JuliaRobotics/IncrementalInference.jl/actions/workflows/ci.yml
[iif-ver-img]: https://juliahub.com/docs/IncrementalInference/version.svg
[iif-rel-url]: https://github.com/JuliaRobotics/IncrementalInference.jl/releases
[iif-milestones]: https://github.com/JuliaRobotics/IncrementalInference.jl/milestones
[iif-cov-img]: https://codecov.io/github/JuliaRobotics/IncrementalInference.jl/coverage.svg?branch=master
[iif-cov-url]: https://codecov.io/github/JuliaRobotics/IncrementalInference.jl?branch=master
[iif-url]: http://www.github.com/JuliaRobotics/IncrementalInference.jl

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
[tf-stable]: https://img.shields.io/badge/2021Q2-v0.2.x-green.svg
[tf-milestones]: https://github.com/JuliaRobotics/TransformUtils.jl/milestones
[tf-releases]: https://github.com/JuliaRobotics/TransformUtils.jl/releases

<!-- | [DrakeVisualizer.jl][dvis-url] | [![Build Status][dvis-build-img]][dvis-build-url] | [![codecov.io][dvis-cov-img]][dvis-cov-url] |
[dvis-cov-img]: https://codecov.io/github/rdeits/DrakeVisualizer.jl/coverage.svg?branch=master
[dvis-cov-url]: https://codecov.io/github/rdeits/DrakeVisualizer.jl?branch=master
[dvis-build-img]: https://travis-ci.org/rdeits/DrakeVisualizer.jl.svg?branch=master
[dvis-build-url]: https://travis-ci.org/rdeits/DrakeVisualizer.jl
[dvis-url]: http://www.github.com/rdeits/DrakeVisualizer.jl -->

[mani-cov-img]: http://codecov.io/github/JuliaManifolds/Manifolds.jl/coverage.svg?branch=master
[mani-cov-url]: https://codecov.io/gh/JuliaManifolds/Manifolds.jl/
[mani-build-img]: https://github.com/JuliaManifolds/Manifolds.jl/workflows/CI/badge.svg
[mani-build-url]: https://github.com/JuliaManifolds/Manifolds.jl/actions?query=workflow%3ACI+branch%3Amaster
[mani-url]: http://www.github.com/JuliaManifolds/Manifolds.jl
[mani-stable]: https://img.shields.io/badge/2021Q2-v0.5.x-green.svg
[mani-releases]: https://github.com/JuliaManifolds/Manifolds.jl/releases

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
[dfg-stable]: https://img.shields.io/badge/2021Q2-v0.14.x-green.svg
[dfg-milestones]: https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/milestones
[dfg-releases]: https://github.com/JuliaRobotics/DistributedFactorGraphs.jl/releases

[amp-cov-img]: https://codecov.io/github/JuliaRobotics/ApproxManifoldProducts.jl/coverage.svg?branch=master
[amp-cov-url]: https://codecov.io/github/JuliaRobotics/ApproxManifoldProducts.jl?branch=master
[amp-build-img]: https://github.com/JuliaRobotics/ApproxManifoldProducts.jl/actions/workflows/ci.yml/badge.svg
[amp-build-url]: https://github.com/JuliaRobotics/ApproxManifoldProducts.jl/actions/workflows/ci.yml
[amp-url]: http://www.github.com/JuliaRobotics/ApproxManifoldProducts.jl
[amp-stable]: https://img.shields.io/badge/2021Q3-v0.4.x-green.svg
[amp-milestones]: https://github.com/JuliaRobotics/ApproxManifoldProducts.jl/milestones
[amp-releases]: https://github.com/JuliaRobotics/ApproxManifoldProducts.jl/releases

[optim-cov-img]: http://codecov.io/github/JuliaNLSolvers/Optim.jl/coverage.svg?branch=master
[optim-cov-url]: https://codecov.io/gh/JuliaNLSolvers/Optim.jl/
[optim-build-img]: https://github.com/JuliaNLSolvers/Optim.jl/actions/workflows/linux.yml/badge.svg
[optim-img-windows]: https://github.com/JuliaNLSolvers/Optim.jl/actions/workflows/windows.yml/badge.svg
[optim-build-url]: https://github.com/JuliaNLSolvers/Optim.jl/actions/workflows/linux.yml
[optim-build-windows]: https://github.com/JuliaNLSolvers/Optim.jl/actions/workflows/windows.yml
[optim-url]: http://www.github.com/JuliaNLSolvers/Optim.jl
[optim-stable]: https://img.shields.io/badge/2021Q3-v1.4.x-green.svg
[optim-releases]: https://github.com/JuliaNLSolvers/Optim.jl/releases

[fsm-cov-img]: https://codecov.io/github/JuliaRobotics/FunctionalStateMachine.jl/coverage.svg?branch=master
[fsm-cov-url]: https://codecov.io/github/JuliaRobotics/FunctionalStateMachine.jl?branch=master
[fsm-build-img]: https://travis-ci.org/JuliaRobotics/FunctionalStateMachine.jl.svg?branch=master
[fsm-build-url]: https://travis-ci.org/JuliaRobotics/FunctionalStateMachine.jl
[fsm-url]: http://www.github.com/JuliaRobotics/FunctionalStateMachine.jl
[fsm-stable]: https://img.shields.io/badge/2020Q3-v0.2.x-green.svg
[fsm-milestones]: https://github.com/JuliaRobotics/FunctionalStateMachine.jl/milestones
[fsm-releases]: https://github.com/JuliaRobotics/FunctionalStateMachine.jl/releases

[rp-url]: http://www.github.com/JuliaRobotics/RoMEPlotting.jl
[rp-cov-img]: https://codecov.io/github/JuliaRobotics/RoMEPlotting.jl/coverage.svg?branch=master
[rp-cov-url]: https://codecov.io/github/JuliaRobotics/RoMEPlotting.jl?branch=master
[rp-build-img]: https://travis-ci.org/JuliaRobotics/RoMEPlotting.jl.svg?branch=master
[rp-build-url]: https://travis-ci.org/JuliaRobotics/RoMEPlotting.jl
[rp-stable]: https://img.shields.io/badge/2021Q2-v0.7.x-green.svg
[rp-milestones]: https://github.com/JuliaRobotics/RoMEPlotting.jl/milestones
[rp-releases]: https://github.com/JuliaRobotics/RoMEPlotting.jl/releases
