module Sequences

using Interpolations

# export
#     MeasurementSequence,
#     getOverlap,
#     ssd,
#     displace

struct MeasurementSequence{A1 <: AbstractArray, A2 <: AbstractArray, T}
    x::A1
    y::A2
    _interpolate::T
end

MeasurementSequence(x::AbstractVector{<:Real},y::AbstractVector{<:Real}) = MeasurementSequence(x,y, LinearInterpolation(x, y))

(ms::MeasurementSequence)(xval) = ms._interpolate(xval)


function getOverlap(s1::MeasurementSequence, s2::MeasurementSequence)   
    (max(minimum(s1.x),minimum(s2.x)), min(maximum(s1.x),maximum(s2.x)))
end

# FIXME BUT NOT ON FIRE (EMBERS), make `res` something on order of incoming sequence grids (and for parent)
function ssd(s1::MeasurementSequence, s2::MeasurementSequence;res::Float64=0.1)
    cr = getOverlap(s1, s2)
    # correlation will be calculated on new grid specific to each overlap
    x = cr[1]:res:cr[2]
    # interpolate both scalar sequences to new grid

    return (1.0/length(x)) * sum((s1.(x) - s2.(x)).^2)
end

function displace(s::MeasurementSequence, d::Float64)
    MeasurementSequence(s.x .+d, s.y)
end

# _ssdCorr(s1,s2,x) = ssd(s1,displace(s2,x))

end