export
    ZmqServer

mutable struct ZmqServer{T <: DFG.AbstractDFG}
    fg::T
    config::Dict{String, Any}
    isServerActive::Bool
    binding::String
end
