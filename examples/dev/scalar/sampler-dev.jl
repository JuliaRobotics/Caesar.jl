using Images
using ImageView
using FileIO
using Interpolations
using Caesar
using RoME
using RoME: MeanMaxPPE
using ProgressMeter
using ImageView
using Statistics

using TensorCast
using UnicodePlots
# using Gadfly
# Gadfly.set_default_plot_size(35cm, 25cm)


prjPath = dirname(dirname(pathof(Caesar)))
include( joinpath(prjPath, "examples","dev","scalar","CommonUtils.jl") )


##

# function getLevelSetSigma(  data::AbstractMatrix{<:Real},
#                             level::Real,
#                             sigma::Real,
#                             x_grid::AbstractVector{<:Real}, 
#                             y_grid::AbstractVector{<:Real};
#                             sigma_scale::Real=3  )
#     """
#     Get the grid positions at the specified height (within the provided spreads)
#     """
#     # make Gaussian
#     roi = data .- level
#     roi .^= 2
#     roi .*= 0.5*(1/(sigma_scale*sigma))^2

#     # truncate at sigma_scale*sigma
#     mask = roi .< 1

#     idx2d = findall(mask)  # 2D indices
#     pos = (v->[x_grid[v[1]],y_grid[v[2]]]).(idx2d)
#     weights = (v->roi[v[1],v[2]]).(idx2d)
#     weights ./= sum(weights)

#     # recast to the appropriate shape
#     @cast kp[i,j] := pos[j][i]
#     collect(kp), weights
# end

# function fitKDE(support,
#                 weights,
#                 x_grid::AbstractVector{<:Real}, 
#                 y_grid::AbstractVector{<:Real};
#                 bw_factor::Real=0.7  )
#     #
#     # 1. set the bandwidth 
#     x_spacing = Statistics.mean(diff(x_grid))
#     y_spacing = Statistics.mean(diff(y_grid))
#     kernel_ = bw_factor*0.5*(x_spacing + y_spacing) # 70% of the average spacing
#     kernel_bw = [kernel_; kernel_]                  # same bw in x and y
#     # fit KDE
#     kde!(support, kernel_bw, weights)
# end

# function sampleHeatmap( data::AbstractMatrix{<:Real}, 
#                         x, 
#                         y, 
#                         level, 
#                         sigma=0.01, 
#                         N::Int=10000 )
#     #
#     support, weights = getLevelSetSigma(data, level, sigma, x, y)
#     P = fitKDE(support, weights, x, y)
#     sample(P, N)[1]
# end

##




##

x, y, dem = CommonUtils.getSampleDEM()

# imshow(dem)


##

# z_elevation = 0.5,      # actual measurement
# sigma_elevation = 0.001 # measurement uncertainty
hmd = HeatmapDensityRegular(dem, (x,y), 0.5, 0.001)

# pr = Mixture(PriorPoint2, [hmd; MvNormal([0;0], [....])], [0.5;0.5])
# addFactor!(fg, [:x17;], pr)


##

pts = rand(hmd, 10000)


# pseudo code
# getSample(hmd, 100, hint_belief)


##

imscatter(img) = scatterplot(img[2,:], -img[1,:])
imdensity(img) = densityplot(img[2,:], -img[1,:])
imscatter(pts)
imdensity(pts)