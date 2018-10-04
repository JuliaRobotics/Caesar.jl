mutable struct Normal                # a Julia composite type generated from protoc
    mean::Float64
    std::Float64
    Normal(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end
