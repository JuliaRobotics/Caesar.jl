module Sequences

using Interpolations
using Statistics

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

function MeasurementSequence(x::AbstractVector{<:Real},y::AbstractVector{<:Real}) 
    x_ = x |> deepcopy
    y_ = y |> deepcopy
    # flip both x and y if x is decreasing
    # @show Statistics.median(diff(x)) 
    if Statistics.median(diff(x)) < 0
        reverse!(x_)
        reverse!(y_)
    end

    MeasurementSequence(x, y, LinearInterpolation(x_, y_))
end


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

    # @info length(x) any(isnan.(x)) any(isnan.(s1.(x))) any(isnan.(s2.(x))) ((1.0/length(x)) * sum((s1.(x) - s2.(x)).^2))
    return (1.0/length(x)) * sum((s1.(x) - s2.(x)).^2)
end

function displace(s::MeasurementSequence, d::Float64)
    MeasurementSequence(s.x .+d, s.y)
end

# _ssdCorr(s1,s2,x) = ssd(s1,displace(s2,x))


# southerly
# a_o = 20
# b_o = 15

# overlap_gt = (15,10) # relative
# overlap_gt = (10,15) # absolute


end