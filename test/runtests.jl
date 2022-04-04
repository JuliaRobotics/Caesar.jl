# using RoMEPlotting
# using IncrementalInference, RoME
using Test

## See tests in DFG, IIF, and RoME for more complete coverage of features

# specific end-to-end checks for ICRA 2022 tutorials
include("ICRA2022_tests/simple_graph.jl")

# highly multipackage tests that don't fit well in specific library dependencies.
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
