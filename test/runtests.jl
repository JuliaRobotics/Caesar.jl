# using RoMEPlotting
using Caesar, Caesar.ZmqCaesar
using IncrementalInference, RoME
using Test

## See tests in DFG, IIF, and RoME for more complete coverage of features

@testset "ZMQ Interface" begin
    # Unit tests
    include("multilangzmq/callbackCompatibilityTest.jl")
    # Integration tests
    include("multilangzmq/zmqInternal.jl")
    # End to end tests
    # Do this manually...
    # include("multilangzmq/serverTest.jl")
end


