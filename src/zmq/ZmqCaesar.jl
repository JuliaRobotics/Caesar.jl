module ZmqCaesar
    using Requires
    using ZMQ
    using JSON
    using Distributions, IncrementalInference, DistributedFactorGraphs, Caesar
    using Unmarshal
    using Dates

    # import ZMQ: ls

    include("models/distributions.jl")
    include("models/factors.jl")
    include("models/config.jl")
    include("models/session.jl")
    include("models/plotting.jl")
    include("models/additional.jl")
    include("models/ZmqServer.jl")
    include("services/factors/distributionSerialization.jl")
    include("services/factors/factorSerialization.jl")
    include("services/config.jl")
    include("services/session.jl")
    include("services/additional.jl")
    include("services/ZmqServer.jl")
    
    # conditional loading for RoMPlotting
    function __init__()
        @require RoMEPlotting="238d586b-a4bf-555c-9891-eda6fc5e55a2" include("services/plotting.jl")
    end
end
