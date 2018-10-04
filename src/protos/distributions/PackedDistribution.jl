mutable struct PackedDistribution                # a Julia composite type generated from protoc
    distType::String
    packed::Vector{UInt8}
    PackedDistribution(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end
