

using ZMQ
using Caesar, Caesar.ZmqCaesar

@testset "ZMQ Interface" begin
    # Unit tests
    include("multilangzmq/callbackCompatibilityTest.jl")
    # Integration tests
    include("multilangzmq/zmqInternal.jl")
    # End to end tests
    # Do this manually...
    # include("multilangzmq/serverTest.jl")
end