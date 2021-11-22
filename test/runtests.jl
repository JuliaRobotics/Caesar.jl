# using RoMEPlotting
# using IncrementalInference, RoME
using Test

## See tests in DFG, IIF, and RoME for more complete coverage of features

include("pcl/testPointCloud2.jl")

include("testPose2AprilTag4Corner.jl")


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


# include("testScatterAlignPose2.jl")
