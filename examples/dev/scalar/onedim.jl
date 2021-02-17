#= 
Basic example of 1D localization against a scalar field
=#

using Images, FileIO           # load elevation from PNG
using DataInterpolations       # handles the terrain
using Distributions
using ImageFiltering
using Plots
gr()

## Terrain
# load dem (18x18km span)
img = load("dev/scalar/dem.png")
h = 1e3* Float64.( @view img[512,:])
# x = range(-9000,9000,length = length(h))
x = range(-100,100,length = length(h))
terrain = LinearInterpolation(h, x)
    plot(r)

## Measurement likelihood
function measurementLikelihood(y, sigma_y,
                               terrain::LinearInterpolation)
    meas_model = Normal(y, sigma_y)
    l = pdf.(meas_model, terrain.u)
    return l./sum(l)
end

##
function propagate(prior, model)
    imfilter(prior, reflect(centered(model)), Fill(0) )
end

function neval(dist, x)
    d = pdf.(dist, x)
    d./sum(d)
end

xt = -70    # true position
delta = 10; # motion increment
sigma_y = 5 # altitude measurement uncertainty
motion_kernel = neval(Normal(delta, 1), x)
p = neval(Normal(xt, 1), x )  # prior

##
using RoME, Caesar

struct ScalarField1D
    # scalar field over one variable f:R->R (i.e. h= f(x))
    # [a special case of ScalarField, h = f(·)]
    # ...
end

# create empty factor graph
fg = initfg()

# add terrain to factor graph
# addConstant!(fg, :terrain, ScalarField1D(dem))

# simulate for a few poses
for i=1:10
    xt = xt + delta   # true position
    yt = terrain(xt)  # true elevation

    p = propagate(p, motion_kernel) # prior
    plot!(x,p)

    addVariable!(fg, Symbol("x$(i-1)"), ContinuousEuclid{1})

    l = measurementLikelihood(yt, sigma_y, terrain)

    p = p.*l # post = prior * likelihood
    p = p./sum(p) # normalize

    # em = ScalarFieldMeasurement(x,l)
    # addFactor!(fg, [:terrain, Symbol("y$(i-1)"), em)
end
