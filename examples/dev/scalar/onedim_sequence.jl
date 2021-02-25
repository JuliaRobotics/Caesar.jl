#= 
Basic example of 1D localization against a scalar field - measurement sequence
=#

using Images, FileIO           # load elevation from PNG
using DataInterpolations       # handles the terrain
using Distributions
using ImageFiltering
using Plots
using Caesar, RoME
using DSP
gr()

## Terrain
# load dem (18x18km span)
img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples/dev/scalar/dem.png")) .|> Gray
h = 1e3*Float64.( @view img[512,:])
# x = range(-9000,9000,length = length(h))
x = range(-100,100,length = length(h))
terrain = LinearInterpolation(h, x)
plot(terrain)

## Measurement likelihood
function measurementLikelihood( meas_model::SamplableBelief,
                                terrain::LinearInterpolation)
    # meas_model = Normal(y, sigma_y)
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


# struct ScalarField1D
#     # scalar field over one variable f:R->R (i.e. h= f(x))
#     # [a special case of ScalarField, h = f(·)]
#     # ...
# end



## implements a scalar measurement factor (e.g. elevation against known scalar map)
# addFactor!(fg, [:x1, :topography], ScalarFieldFactor(y))
# FIXME: objective is to <: IIF.AbstractRelativeConstant (final name TBD)
struct ScalarFieldSequenceFactor1D <: IIF.AbstractPrior
  gridSequence::Vector{Float64} # sample location (odometry)
  scalarSequence::Vector{Float64} #sample value (elevation meas)
end

function IIF.getSample(s::CalcFactor{<:ScalarFieldSequenceFactor1D}, N::Int=1)
    # locality if desired from current variable estimate, 
    # X0 = getBelief(s.metadata.fullvariables[1]) # to consider only local map info

    # assuming user addConstant!(fg, :terrain, ScalarField1D(dem))

    global terrain # should come from s.constants[:terrain] ?????

    # ensure equal spacing as terrain
    # assumptions:
    # - terrain.t is equally spaced
    # - s.factor.gridSequence increases monotonically
    measurement= LinearInterpolation(s.factor.scalarSequence,s.factor.gridSequence)
    dx = diff(terrain.t)[1]
    template = measurement.(0:dx:s.factor.gridSequence[end])
    
    intensity_ = xcorr([terrain.u;], template, padmode=:none)
    chop_ = length(template)-1
    intensity = intensity_[chop_:(end-chop_)]

    # AliasingSS only works for 1D at this time
    # NOTE, if you stray out of region of support, things blow up!
    bss = AliasingScalarSampler(terrain.t, intensity)
    
    return (reshape(rand(bss,N),1,N),)
end

# # keep track of IIF #1051
# # parameters are both in map/trajectory space X
# function (s::CalcFactor{<:ScalarFieldSequenceFactor1D})(likelihoodSample, pose)
#     return likelihoodSample - pose
# end

##


# create empty factor graph
fg = initfg()
# getSolverParams(fg).inflation = 3.0

addVariable!(fg, :x0, ContinuousEuclid{1})

# add terrain to factor graph
# addConstant!(fg, :em, ScalarField1D(dem))

# simulate for a few poses
xt = -70
delta = 2 # pose increment
sigma_x = 1.0 # odometry measurement uncertainty
sigma_y = 1.0 # elevation measurement uncertainty

for i=1:10
    x_seq = zeros(0)  # reset measurement sequence
    y_seq = zeros(0)
    for j=1:seq_length
        xt = xt + delta   # true position
        yt = terrain(xt)  # true elevation

        # append to sequence
        append!(x_seq, xt + sigma_x*randn(1))
        append!(y_seq, yt + sigma_y*randn(1))
    end

    addVariable!(fg, Symbol("x$(i)"), ContinuousEuclid{1}) # BoundedEuclid{1}

    # add propagation
    addFactor!(fg, [Symbol("x$j") for k in (i-1):i], LinearRelative(Normal(delta*seq_length, 5)),graphinit=false)

    em = ScalarFieldSequenceFactor1D( x_seq-x_seq[1], y_seq ) # sigma_y
    # addFactor!(fg, [:terrain, Symbol("y$(i-1)")], em)
    # the hacky version
    addFactor!(fg, [Symbol("x$(i)");], em, graphinit=false)
end


## talk some solving

# still experimental, treeinit
getSolverParams(fg).graphinit = false
getSolverParams(fg).treeinit = true


solveTree!(fg)

##


smplOnMap = approxConv(fg, :x10f1, :x10)


histogram(smplOnMap[:], bins=100)

plot(terrain)
plot!([-100; 100],[436; 436])



##

using RoMEPlotting
Gadfly.set_default_plot_size(25cm,20cm)

##


pl1 = plotKDE(fg, sortDFG(ls(fg)))

pl1.coord = Coord.cartesian(xmin=-100,xmax=85)

pl2 = Gadfly.plot(x=terrain.t,y=terrain.u, Geom.line)


vstack(pl1, pl2)


##