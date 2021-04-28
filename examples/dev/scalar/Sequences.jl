module Sequences

using Interpolations

# export
#     MeasurementSequence,
#     getOverlap,
#     ssd,
#     displace

struct MeasurementSequence
    x::AbstractArray
    y::AbstractArray
end

function getOverlap(s1::MeasurementSequence, s2::MeasurementSequence)   
    (max(minimum(s1.x),minimum(s2.x)), min(maximum(s1.x),maximum(s2.x)))
end

function ssd(s1::MeasurementSequence, s2::MeasurementSequence;res::Float64=0.1)
    cr = getOverlap(s1, s2)
    x = cr[1]:res:cr[2]
    y1c = LinearInterpolation(s1.x, s1.y)
    y2c = LinearInterpolation(s2.x, s2.y)

    return (1.0/length(x))*sum(y1c(x)-y2c(x)).^2
end

function displace(s::MeasurementSequence, d::Float64)
    MeasurementSequence(s.x .+d, s.y)
end

end