# Incorporating Neural Network Factors

IncrementalInference.jl and RoME.jl has native support for using Neural Networks (via [Flux.jl](https://fluxml.ai/Flux.jl/stable/)) as non-Gaussian factors.  Documentation is forthcoming, but meanwhile [see the following generic Flux.jl factor structure](https://github.com/JuliaRobotics/IncrementalInference.jl/tree/master/src/Flux).  Note also that a standard [`Mixture` approach already exists too](https://github.com/JuliaRobotics/RoME.jl/blob/master/src/factors/flux/MixtureFluxPose2Pose2.jl).

