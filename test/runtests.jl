using Caesar
using IncrementalInference, RoME
using Base.Test

@testset "ZMQ Interface" begin
    # Unit tests
    include("multilangzmq/callbackCompatibilityTest.jl")
    # Integration tests
    include("multilangzmq/zmqInternal.jl")
    # End to end tests
    include("multilangzmq/serverTest.jl")
end

# println("[TEST] CloudGraphs API calls...")
# if false
#   println("[TEST] with CloudGraphs with local DB data layer (multicore)...")
#   include("fourdoortestcloudgraph.jl")
#   println("[SUCCESS]")
# else
#   warn("[NOT TESTING] CloudGraphs interface, auth required -- you can enable it here at $(@__FILE__).")
# end


# if false
#   # using VictoriaParkTypes
#
# else
#   warn("not running full Victoria park dataset now")
# end
