#= 
Basic example of 1D localization against a scalar field - measurement sequence
=#

using Images, FileIO           # load elevation from PNG
using DataInterpolations       # handles the terrain
using Distributions
using ImageFiltering
using Plots
using Gadfly, Colors
using Statistics
using Caesar, RoME
using DSP
gr()

## Terrain
# load dem (18x18km span)
img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples/dev/scalar/dem.png")) .|> Gray
h = 1e3*Float64.( @view img[512,:])

## Size of the map
# x = range(-9000,9000,length = length(h))
x = range(-100,100,length = length(h))
terrain = LinearInterpolation(h, x)
Plots.plot(terrain)

## Measurement likelihood
# function measurementLikelihood( meas_model::SamplableBelief,
#                                 terrain::LinearInterpolation)
#     # meas_model = Normal(y, sigma_y)
#     l = pdf.(meas_model, terrain.u)
#     return l./sum(l)
# end

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

##

global lasttimeisacharm

# assumes map and template are on equivalent grid
# this function does not care for where/what the grid is
# Note: result will assum base pose is first in sequence
# for each i
#   energy[i] = sum( (map[i:(i+length(template))] - template).^2 )
# density = exp.(-energy)     # Woodward
function _mySSDCorr(map, template)
    global lasttimeisacharm
    len = length(template)
    mnm = Statistics.mean(map)
    map_ = [map; mnm*ones(len-1)]
    # adaptive = sum((template).^2)
    density = zeros(length(map))
    for i in 1:length(map)
        density[i] = -sum( (map_[i:(i+len-1)] .- template).^2 )
    end
    adaptive = abs(maximum(density))

    density ./= adaptive
    density .= exp.(density)
    density ./= sum(density)
    lasttimeisacharm = density
    # density ./= sum(exp.(density))
    return density
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
    # s.factor.gridSequence is the sequence of odometry estimates at which
    #  the elevation measurements (s.factor.scalarSequence) were taken
    measurement = LinearInterpolation(s.factor.scalarSequence,s.factor.gridSequence)
    dx = diff(terrain.t)[1]
    template = measurement.(s.factor.gridSequence[1]:dx:s.factor.gridSequence[end])

    intensity_ = _mySSDCorr([terrain.u;], template)
    
        # intensity_ = xcorr([terrain.u;], template, padmode=:none)
        # chop_ = round(Int, length(template)/2)
        # intensity = intensity_[chop_:(end-chop_)]
        # # force equal length
        # while length(intensity) < length(terrain.t)
        #     push!(intensity, 0)
        # end
        # intensity = intensity[1:length(terrain.t)]

    # AliasingSS only works for 1D at this time
    # NOTE, if you stray out of region of support, things blow up!
    bss = AliasingScalarSampler([terrain.t;], intensity_)
    
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

# number of interpose measurements (will get upsampled to the grid size!)
seq_length = 10

# simulate for a few poses
xt = -70
delta = 1 # pose increment
sigma_x = 0.1 # odometry measurement uncertainty
sigma_y = 1.0 # elevation measurement uncertainty

xt_seq = zeros(0)
yt_seq = zeros(0)
x_seq = zeros(0)  
y_seq = zeros(0)

for i=1:10
    xt_seq = zeros(0)
    yt_seq = zeros(0)
    x_seq = zeros(0)  
    y_seq = zeros(0)

    # build up sequence
    for j=1:seq_length
        xt = xt + delta         # true position
        yt = terrain(xt)  # true elevation

        append!(xt_seq,xt)
        append!(yt_seq,yt)

        # append to sequence
        append!(x_seq, xt .+ sigma_x*randn(1))
        append!(y_seq, yt .+ sigma_y*randn(1))
    end

    addVariable!(fg, Symbol("x$(i)"), ContinuousEuclid{1}) # BoundedEuclid{1}

    # add propagation
    fctlbls = [Symbol("x$j") for j = (i-1):i]
    addFactor!(fg, fctlbls, LinearRelative(Normal(delta*seq_length, 5)),graphinit=false)

    # grid, scalar
    em = ScalarFieldSequenceFactor1D( x_seq , y_seq )  # .- x_seq[1]

    # addFactor!(fg, [:terrain, Symbol("y$(i-1)")], em)
    # the hacky version
    addFactor!(fg, [Symbol("x$(i)");], em, graphinit=false)
end


## test the correlator prior

# eval the sampler at first prior

PL = []
for lb in (ls(fg) |> sortDFG)[2:end]
    pts = approxConv(fg, Symbol(lb, "f1"), lb)
    prop = kde!(pts)
    pl = plotKDE(prop)

    pl2 = Gadfly.plot(
        Gadfly.layer(x=terrain.t, y=terrain.u, Geom.line, Theme(default_color=colorant"red")),
        Gadfly.layer(x=xt_seq,y=yt_seq, Geom.line, Theme(default_color=colorant"green")),
        Gadfly.layer(x=x_seq, y=y_seq, Geom.line, Theme(default_color=colorant"blue"))
    )

    push!(PL, vstack(pl, pl2))
end

##

PL[10]

##


## plot true and measured values (last sequence only)
plot(terrain.t, terrain.u)
plot!(xt_seq,yt_seq)
plot!(x_seq,y_seq)

# plot sample correlator output



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