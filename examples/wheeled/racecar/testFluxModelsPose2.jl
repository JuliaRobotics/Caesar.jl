# test FluxModelsPose2Pose2

using LinearAlgebra
using Flux
using RoME

import RoME: FluxModelsPose2Pose2

##

include(joinpath((@__DIR__), "LoadPyNNTxt.jl"))

allModels = []
for i in 0:99
# /home/dehann/data/racecar/results/conductor/models/retrained_network_weights0
  push!(allModels, loadTfModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/models/retrained_network_weights$i") )
end

##

# start with a basic factor graph
fg = generateCanonicalFG_ZeroPose2()

mvnNaive = MvNormal(zeros(3), diagm([1.0;1.0;0.01]))

addVariable!(fg, :x1, Pose2)

jvd = zeros(25,4)
pp = FluxModelsPose2Pose2(allModels, jvd, mvnNaive, 0.5)

addFactor!(fg, [:x0;:x1], pp)


pts = approxConv(fg, :x0x1f1, :x1)

#
