```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/6412556/134777705-acda768e-884b-4de6-b307-aac6d09b9c81.png" width="240" border="0" />
</p>
```

## Introduction

Caesar.jl facilates software development for spatial/physical AI from multiple sensor data, and multiple sessions or human / semi-autonomous / autonomous agents.  This software is being developed with broadly [Industry 4.0](https://en.wikipedia.org/wiki/Fourth_Industrial_Revolution), AI, data analytics, robotics, and [Work of the Future](https://workofthefuture.mit.edu/) in mind.  Caesar.jl is an "umbrella package" to combine many other libraries from across the Julia package ecosystem.  Additionally, the [NavAbilitySDKs](https://github.com/NavAbility/) allow for multi-language and cost-sharing features.  Commercial support is available where large networks, multiple compute loads, and expert help can help drastically reduce cash-burn.

## TL;DR; Science Underpinning

Caesar.jl is primarilly concerned with data processing / algorithmic / AI-related software for data-fusion, perception, contradiction extraction, navigation affordances, surrogates, same-day-GPS-replacement, geometric representation, sensor calibration, solution verification, efficient solving, equipment interactions, reality capture, digital twins, swarms, and related topics.  At the highest level, the entire purpose of CJL and co are for the most engineering sensible and power solutions to the following expressions:
```
# the "measurement process/data collection manifestation/reality/collapse" is (i.e. robot drives around somewhere)
observation_states + model_uncertainty = convolve(physics_models, observation_uncertainty)
# model_uncertainty = quantum_uncertainty + measurement_error

# while the "solve graph" manifestation (aka inversion / data-fusion / state estimation / inference) is:
estimated_states + model_uncertainty = deconvolve(best_models, observation_states)
# model_uncertainty = (quantum + modeling_error) + numerical + ...
```

## Open-core, Community and Commercial

Click here to go to the Caesar.jl (permisively licensed) Github repo:

[![source](https://img.shields.io/badge/source-code-yellow.svg)](https://github.com/JuliaRobotics/Caesar.jl)

[WhereWhen.ai's NavAbility products and services](https://www.wherewhen.ai) continues to develop the Caesar.jl suite of open-source libraries.  Please reach out for any additional information (info@navability.io), or using the community links provided below.

Various mapping and localization solutions are possible both for commercial and R&D.  We recommend taking a look at:
- The human-to-machine friendly [NavAbility App](https://app.navability.io/home/) interaction; and
- The machine-to-machine friendly NavAbilitySDKs ([Python](https://github.com/NavAbility/NavAbilitySDK.py), [Julia](https://github.com/NavAbility/NavAbilitySDK.jl), [Rust/C/Py/JS](https://github.com/NavAbility/NavAbilitySDK.rs), etc.).  Also see the [SDK.py Docs](https://navability.github.io/NavAbilitySDK.py/).

## NavAbility Zero Install Tutorials

Follow [this page to see the **NavAbility Tutorials**](https://navability.github.io/NavAbilitySDK.py/nvatutorials.html) which are zero install and build around specific application examples.

# Origins and Ongoing Research

Caesar.jl developed as a spin-out project from MIT's Computer Science and Artificial Intelligence Laboratory.  See related works on [the literature page](https://www.juliarobotics.org/Caesar.jl/latest/refs/literature/).  Many future directions are in the works -- including fundamental research, implementation quality/performance, and system integration.

Consider citing our work: [CITATION.bib](https://github.com/JuliaRobotics/Caesar.jl/blob/master/CITATION.bib).

## Community, Issues, Comments, or Help

Post [Issues](https://github.com/JuliaRobotics/Caesar.jl/issues), or [Discussions](https://github.com/JuliaRobotics/Caesar.jl/discussions) for community help.  Maintainers can easily transfer Issues to the best suited package location if necessary.  Also see the history of changes and ongoing work can via the [Milestone pages (click through badges here)](https://github.com/JuliaRobotics/Caesar.jl/blob/master/README.md#bleeding-edge-development-status).  You can also get in touch via Slack at [![](https://img.shields.io/badge/Invite-Slack-green.svg?style=popout)](https://join.slack.com/t/caesarjl/shared_invite/zt-ucs06bwg-y2tEbddwX1vR18MASnOLsw).

!!! note
    Please help improve this documentation--if something confuses you, chances
    are you're not alone. It's easy to do as you read along: just click on the
    "Edit on GitHub" link above, and then
    [edit the files directly in your browser](https://help.github.com/articles/editing-files-in-another-user-s-repository/).
    Your changes will be vetted by developers before becoming permanent, so don't
    worry about whether you might say something wrong.

# JuliaRobotics Code of Conduct

The Caesar.jl project is part of the JuliaRobotics organization and adheres to the JuliaRobotics [code-of-conduct](https://github.com/JuliaRobotics/administration/blob/master/code_of_conduct.md).
# Next Steps
For installation steps, examples/tutorials, and concepts please refer to the following pages:

```@contents
Pages = [
    "concepts/why_nongaussian.md"
    "installation_environment.md"
    "concepts/concepts.md"
    "concepts/building_graphs.md"
    "concepts/2d_plotting.md"
    "examples/examples.md"
]
Depth = 1
```

