using IncrementalInference

"""
    ScalarPointToMultiPoint

Factor modeling an uncertain association between a point and a set of other 
points. Measurement model operates on the difference between the measured
field value at the point and the interpolated field value (from the point 
set).
"""
struct ScalarPointToMultiPoint{T<:SamplableBelief}
    # nothing
end

function IIF.getSample(s::CalcFactor{<:ScalarPointToMultiPoint}, N::Int=1)
    # generate 0-mean 1D  samples
    return (reshape(rand(s.factor.Z, N), 1, N), )
end

function (s::CalcFactor{<:ScalarPointToMultiPoint})(point, points)
    # calc difference between field value at point and 
    # interpolated field value from points

    h = Interpolations.LinearInterpolation((points[1]..., points[2]...),points[3]...)

    return point[3] - h(point[1],point[2])
end

