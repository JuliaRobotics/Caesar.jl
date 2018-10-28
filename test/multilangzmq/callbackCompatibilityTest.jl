using Caesar, Caesar.ZmqCaesar
using Test

@testset "ZMQ Callback Compatibility Tests" begin

    # Bring in the callbacks symbols
    include(joinpath(dirname(@__FILE__), "..", "..", "src", "zmq", "services", "ZmqServer.jl"))

    @testset "ZMQ Callback Compatibility Tests: System Callbacks" begin
        for callback in systemverbs
            @test typeof(getfield(Caesar.ZmqCaesar, callback)) <: Function
        end
    end

    @testset "ZMQ Callback Compatibility Tests: Config Callbacks" begin
        for callback in configverbs
            @test typeof(getfield(Caesar.ZmqCaesar, callback)) <: Function
        end
    end

    @testset "ZMQ Callback Compatibility Tests: Session Callbacks" begin
        for callback in sessionverbs
            @test typeof(getfield(Caesar.ZmqCaesar, callback)) <: Function
        end
    end
end
