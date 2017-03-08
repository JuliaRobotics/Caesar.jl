using Caesar
using IncrementalInference, RoME
using HDF5, JLD
using Base.Test

println("[TEST] basics")

as = ArcPointsRangeSolve([0.0;1.0;0.0],[1.0/sqrt(2.0);1.0/sqrt(2.0);0.0],[1.0;0.0;0.0],1.0)
findaxiscenter!(as)
@show as.center, as.axis, as.angle
@test norm(as.center-[0.0;0.0;0.0]) < 1e-4
@test norm(as.axis-[0.0;0.0;-1.0]) < 1e-4
@test -1e-4 < as.angle-pi/2 < 1e-4


as = ArcPointsRangeSolve([2.0;0;0.0],[0.5;1.5;0.0],[-1.0;0;0.0],1.5)
findaxiscenter!(as)
@show as.center, as.axis, as.angle
@test norm(as.center-[0.5;0.0;0.0]) < 1e-4
@test norm(as.axis-[0.0;0.0;1.0]) < 1e-4
@test -1e-4 < as.angle-pi < 1e-4

as = ArcPointsRangeSolve([0;-1.0;0],[0;0.5;1.5],[0;2.0;0.0],1.5)
findaxiscenter!(as)
@show as.center, as.axis, as.angle
@test norm(as.center-[0;0.5;0]) < 1e-4
@test norm(as.axis-[-1.0;0.0;0.0]) < 1e-4
@test -1e-4 < as.angle-pi < 1e-4

println("[SUCCESS]")
