#= 
Proof of concept of 1D localization against a scalar field.
In this example we use a sequence of 1D measurements and a known map with a
correlator-like approach to localize.

2021-03-10_16
=#

using Images, FileIO           # load elevation from PNG
using ImageFiltering
using DSP
using DataInterpolations       # handles the terrain
using Distributions
using Statistics
using Caesar, RoME
using Gadfly, Colors
using RoMEPlotting
using Plots


## Preamble: creating the measurement sequence factor 

# 1. This is the (constant) scalar field the elevation measurements refer to
#
# struct ScalarField1D
#     # scalar field over one variable f:R->R (i.e. h= f(x))
#     # [a special case of ScalarField, h = f(·)]
#     # ...
# end

# 2. This is the actual scalar measurement sequence factor
# It takes two arguments: 
#  - the elevation measurement sequence, and 
#  - the odometry estimate at each elevation measurement
#
# addFactor!(fg, [:x1, :topography], ScalarFieldSequenceFactor1D(x,y))
struct ScalarFieldSequenceFactor1D <: IIF.AbstractPrior
  # FIXME: objective is to <: IIF.AbstractRelativeConstant (final name TBD)
  gridSequence::Vector{Float64} # sample location (odometry)
  scalarSequence::Vector{Float64} #sample value (elevation meas)
end


# 3. Objective/fitness function evaluating match between a map and a template for different displacement values 
# This is used within the factor to generate the likelihood over the terrain
#
# NOTE: this function assumes map and template are on an equivalent grid 
# this function does not care for where/what the grid is
# NOTE: result will assume base pose is first in sequence
# for each i
#   energy[i] = sum( (map[i:(i+length(template))] - template).^2 )
# density = exp.(-energy)     # Woodward
function _mySSDCorr(map, template)
    len = length(template)
    mnm = Statistics.mean(map)
    map_ = [map; mnm*ones(len-1)]
    density = zeros(length(map))
    for i in 1:length(map)
        density[i] = -sum( (map_[i:(i+len-1)] .- template).^2 )
    end
    adaptive = abs(maximum(density))

    density ./= adaptive
    density .= exp.(density)
    density ./= sum(density)
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


## Terrain

# load dem (18x18km span)
img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples/dev/scalar/dem.png")) .|> Gray
h = 1e3*Float64.( @view img[512,:])

## Size of the map
# x = range(-9000,9000,length = length(h)) # actual span is 18km on each axis
x = range(-100,100,length = length(h))
global terrain
terrain = LinearInterpolation(h, x)
Plots.plot(terrain)


## Simulation parameters
seq_length = 10 # measurement sequence length (will get upsampled to the grid size!)
xt = -70        # true pose (initial value)
delta = 1       # pose increment
sigma_x = 0.1   # odometry measurement uncertainty
sigma_y = 1.0   # elevation measurement uncertainty



fg = initfg() # create empty factor graph
# getSolverParams(fg).inflation = 3.0

addVariable!(fg, :x0, ContinuousEuclid{1})  # add first pose

# add terrain to factor graph (missing for now)
# addConstant!(fg, :em, ScalarField1D(dem))

# simulate for a few poses
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
    labels = [Symbol("x$j") for j = (i-1):i]
    addFactor!(fg, labels, LinearRelative(Normal(delta*seq_length, 5)),graphinit=false)

    # grid, scalar
    em = ScalarFieldSequenceFactor1D( x_seq , y_seq )  # .- x_seq[1]

    # addFactor!(fg, [:terrain, Symbol("y$(i-1)")], em)        # The Right Way (TM)
    addFactor!(fg, [Symbol("x$(i)");], em, graphinit=false) # the hacky version
end


## eval the sampler at first prior
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

PL[10]


## Solve 

getSolverParams(fg).graphinit = false
getSolverParams(fg).treeinit = true # still experimental, treeinit

solveTree!(fg)

## Show results

smplOnMap = approxConv(fg, :x10f1, :x10)
histogram(smplOnMap[:], bins=100)

plot(terrain)
plot!([-100; 100],[436; 436])

# all poses
Gadfly.set_default_plot_size(25cm,20cm)
pl1 = plotKDE(fg, sortDFG(ls(fg)))             # plot posteriors for all variables
pl1.coord = Coord.cartesian(xmin=-100,xmax=85)
pl2 = Gadfly.plot(x=terrain.t,y=terrain.u, Geom.line)
vstack(pl1, pl2)

##