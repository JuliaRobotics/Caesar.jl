
struct ZmqServer
    fg::IncrementalInference.FactorGraph
    config::Dict{String, Any}
    isServerActive::Bool
    binding::String
end
