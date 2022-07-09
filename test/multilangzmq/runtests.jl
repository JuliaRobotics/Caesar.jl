

using ZMQ
using Caesar, Caesar.ZmqCaesar

@testset "ZMQ Interface" begin
    # Unit tests
    include("callbackCompatibilityTest.jl")
    # Integration tests
    include("zmqInternal.jl")
    # End to end tests
    # Do this manually...
    # include("serverTest.jl")
end