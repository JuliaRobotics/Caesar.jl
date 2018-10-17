using Caesar, Caesar.ZmqCaesar
using Base.Test

@testset "ZMQ Callback Compatibility Tests" begin

    # Bring in the callbacks symbols
    include(joinpath(Pkg.dir("Caesar"), "src", "zmq", "services", "ZmqServer.jl"))

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
