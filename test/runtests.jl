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


println("[TEST] CloudGraphs API calls...")
# instpkg = Pkg.installed()
if false
  println("[TEST] with CloudGraphs with local DB data layer (multicore)...")
  include("fourdoortestcloudgraph.jl")
  println("[SUCCESS]")
else
  warn("[NOT TESTING] CloudGraphs interface -- you must enable it here at Caesar/test/runtests.jl, and you'll need Neo4j and MongoDB installed.")
end


if false
  # using VictoriaParkTypes

else
  warn("not running full Victoria park dataset now")
end
