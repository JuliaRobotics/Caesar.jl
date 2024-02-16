# using RoMEPlotting
# using IncrementalInference, RoME
using Test

## See tests in DFG, IIF, and RoME for more complete coverage of features

TEST_GROUP = get(ENV, "IIF_TEST_GROUP", "all")

# temporarily moved to start (for debugging)
#...

# functional tests
if TEST_GROUP in ["all", "basic_functional_group"]
    println("Starting tests...")
    # highly multipackage tests that don't fit well in specific library dependencies.
    include("testFolderDict.jl")
    include("testScatterAlignParched.jl")
    include("testScatterAlignPose2.jl")
    include("testScatterAlignPose3.jl")
    include("testStashing_SAP.jl")
    include("pcl/testBoundingBox.jl")
    include("pcl/testPointCloud2.jl")
    @error "Must restore zmq runtest"
    # include("multilangzmq/runtests.jl")
    include("testPose2AprilTag4Corner.jl")
end


# numerical quality tests
if TEST_GROUP in ["all", "test_cases_group"]
    # specific end-to-end checks for ICRA 2022 tutorials
    include("ICRA2022_tests/runtests.jl")
end
