mutable struct PackedPose2Point2BearingRange                # a Julia composite type generated from protoc
    bearing::PackedDistribution
    range::PackedDistribution
    PackedPose2Point2BearingRange(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end
