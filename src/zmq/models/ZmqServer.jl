export
    ZmqServer

mutable struct ZmqServer
    fg::GraphsDFG
    config::Dict{String, Any}
    isServerActive::Bool
    binding::String
end
