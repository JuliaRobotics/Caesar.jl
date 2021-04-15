#= 
Proof of concept of 2D localization against a scalar field.
In this example we use a sequence of 1D measurements and a known map with a
correlator-like approach to localize.

2021-03-24,31
=#

using Images, FileIO           # load elevation from PNG
using ImageFiltering
using DSP
# using DataInterpolations       # handles the terrain
using Interpolations
using Distributions
using Statistics
using Caesar, RoME
using Gadfly, Colors
using Cairo
using RoMEPlotting
using Plots
# using ImageView

Gadfly.set_default_plot_size(35cm,25cm)


## Case 4.
# Square helix northward, with overlapping east/west segments - mapping
# no prior map

# load dem (18x18km span)
img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples/dev/scalar/dem.png")) .|> Gray
img = Float64.(img)
# imshow(img)
# interpolant object for querying
x = range(-9e3, 9e3, length = size(img)[1]) # North
y = range(-9e3, 9e3, length = size(img)[2]) # East
dem = Interpolations.LinearInterpolation((x,y), img)


# not to scale on DEM image, but sequence correlation approach does not matter
terrE_ = 1e3*Float64.(@view img[200,:])
terrW_ = 1e3*Float64.(@view img[300,:])
x = range(-100, 100, length = length(terrE_))

global terrE = Interpolations.LinearInterpolation(x, terrE_)
global terrW = Interpolations.LinearInterpolation(x, terrW_)


function driveLeg(x0, terrain, v, units)
    x = zeros(2,0)
    xm = zeros(2,0)
    z = zeros(0)
    zm = zeros(0)

    for i=1:units
        x.append(x[end]+v)
        xm.append(x[end]+ randn(2,1)) # noisy position
        
        z.append( terrain(x[end]))  # not quite as it isn't 2d interp
        zm.append(z + sigma_z*randn(1))
    end

    # return pose, measurements (valid measured, )
end


##


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
# addFactor!(fg, [:x1, :topography], ScalarFieldSequenceFactorNorthSouth(x,y))
struct ScalarFieldSequenceFactorNorthSouth{T} <: IIF.AbstractRelativeMinimize
  # FIXME: objective is to <: IIF.AbstractRelativeConstant (final name TBD)
  prevSequence::Vector{Float64} # sample location (odometry)
  nextSequence::Vector{Float64} # sample value (elevation meas)
  partial::T
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
# returns vector the size of length(map)
function _mySSDCorr(map, template)

    maplen = length(map)
    len = length(template)
    
    mnm = Statistics.mean(map)
    map_ = [map; mnm*ones(len-1)]
    density = zeros(maplen)
    for i in 1:maplen      
      density[i] = -sum( (map_[i:(i+len-1)] .- template).^2 )
    end
    adaptive = abs(maximum(density))

    density ./= adaptive
    density .= exp.(density)
    density ./= sum(density)
    return density
end


function IIF.getSample(s::CalcFactor{<:ScalarFieldSequenceFactorNorthSouth}, N::Int=1)
  # locality if desired from current variable estimate, 
  # X0 = getBelief(s.metadata.fullvariables[1]) # to consider only local map info

  # assuming user addConstant!(fg, :terrain, ScalarField1D(dem))

  # global terrain # should come from s.constants[:terrain] ?????

  # ensure equal spacing as terrain
  # assumptions:
  # - terrain.t is equally spaced
  # - s.factor.gridSequence increases monotonically
  # s.factor.gridSequence is the sequence of odometry estimates at which
  #  the elevation measurements (s.factor.scalarSequence) were taken
    # measurement = LinearInterpolation(s.factor.scalarSequence,s.factor.gridSequence)

    # upsample = 1 # avoid quantization effects from gridded sequence/terrain data
    # dx = diff(terrain.t)[1]/upsample
    # template = measurement.(s.factor.gridSequence[1]:dx:s.factor.gridSequence[end])
    # map_ = [terrain.u;]
    # map2_grid = range(terrain.t[1],terrain.t[end],length=upsample*length(map_))
    # map2 = terrain.(map2_grid)

  intensity_ = _mySSDCorr(prevSequence, nextSequence )
  
  
  # AliasingSS only works for 1D at this time
  # NOTE, if you stray out of region of support, things blow up!
  bss = AliasingScalarSampler([map2_grid;], intensity_) # [terrain.t;]
      
  # buffer with kde for continuous sampling (not grid spaced)
  stagedsmpls = rand(bss,N)
  smpls = rand(kde!(stagedsmpls, [3*dx;]),N)

  return (reshape(smpls,1,N),)
end


##

nextPose(ps::Symbol; pattern=r"x") = Symbol("x",match(r"\d+", string(ps)).match |> x->(parse(Int,x)+1))

# drive clockwise, x is North, y is East (NED convention).
# boxes start bottom left, spine of boxy helix is on x-axis
function driveOneBox!(fg, 
                      terrain; 
                      lastPose=sortDFG(ls(fg, tags=[:POSE]))[end],
                      start = 0.0, 
                      NS = 15,
                      EW = 15  )
  #
  img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples/dev/scalar/dem.png")) .|> Gray
  h = 1e3*Float64.( @view img[512,:])
  runback = 2/3
  
  # drive North NS units
  newPose = nextPose(lastPose)
  addVariable!(fg, newPose, Point2, tags=[:POSE;:NORTHING])
  # constrain variable in Y (east/west)
  addFactor!(fg, [newPose;], PartialPrior(Normal(0.0, 0.1),(2,)))
  # add odometry constraint
  p2p = Point2Point2(MvNormal([NS; 0.0], [1.0; 1.0]))
  addFactor!(fg, [lastPose; newPose], p2p, tags=[:ODOMETRY;:NORTHING])
  lastPose = newPose

  # drive East EW units
  newPose = nextPose(lastPose)
  addVariable!(fg, newPose, Point2, tags=[:POSE;:EASTING])
  # constrain variable in Y (east/west)
  addFactor!(fg, [newPose;], PartialPrior(Normal(EW, 0.1),(2,)))
  p2p = Point2Point2(MvNormal([0.0, EW], [1.0; 1.0]))
  addFactor!(fg, [lastPose; newPose], p2p, tags=[:ODOMETRY;:EASTING])
  lastPose = newPose
  
  # drive South 0.7NS units
  newPose = nextPose(lastPose)
  addVariable!(fg, newPose, Point2, tags=[:POSE;:SOUTHING])
  # constrain variable in Y (east/west)
  addFactor!(fg, [newPose;], PartialPrior(Normal(EW, 0.1),(2,)))
  p2p = Point2Point2(MvNormal([-runback*NS; 0.0], [1.0; 1.0]))
  addFactor!(fg, [lastPose; newPose], p2p, tags=[:ODOMETRY;:SOUTHING])
  lastPose = newPose
  
  # drive West EW units
  newPose = nextPose(lastPose)
  addVariable!(fg, newPose, Point2, tags=[:POSE;:WESTING])
  # constrain variable in Y (east/west)
  addFactor!(fg, [newPose;], PartialPrior(Normal(0.0, 0.1),(2,)))
  p2p = Point2Point2(MvNormal([0.0; -EW], [1.0; 1.0]))
  addFactor!(fg, [lastPose; newPose], p2p, tags=[:ODOMETRY;:WESTING])
  lastPose = newPose

  # cop out early on first run, or debug
  dofactor ? nothing : (return nothing)

  # Assume adding factors back to PREVIOUS boxy 
  # EAST SIDE

  # build odometry+elevation measurement sequence 
  # (curr_seq: x4 to x5; prev_seq: x0 to x1 => constraint x0 & x4
  # m4: minus four (previous leg); 0: current leg
  x_seq_m4 = LinRange(start,start+NS,100) # assumed 100 measurements per leg
  z_seq_m4 = terrW_.(x_seq_m4)
  # x_seq_m4_ = deepcopy(x_seq_m4) # + noise

  _thirdfwd = (1-runback)*NS
  x_seq0 = LinRange(start+_thirdfwd, start+_thirdfwd+NS, 100) # assumed 100 measurements per leg
  z_seq0 = terrW_.(x_seq0)
  # x_seq0_ = deepcopy(x_seq0) # + noise

  # correlate z_seq0 against z_seq_m4
  intensity = _mySSDCorr(z_seq0, z_seq_m4)

  # get pose four poses back
  # get terrain sequence between (newPose - NW) up to (newPose)
  # curr_seq = terrE(curr_x_seq)
  # prev_seq = terrE(prev_x_seq)
  # # compute SSDcorr wrt sequence 4 poses before
  # # add PartialLinearRelative between the two 
  # intensity = _mySSDCorr(curr_seq, prev_seq)
  
  plr = ScalarFieldSequenceFactorNorthSouth(x_seq_, z_seq, (1,))
  addFactor!(fg, [:x0; :x4], plr, tags=[:TERRAIN,:MATCH])
      # plr = ScalarFieldSequenceFactorNorthSouth(x_seq, z_seq, (1,))
      # # # plr = PartialLinearRelative(grid, AliasingScalarSampler(intensity))
      # addFactor!(fg, [prevPose newPose], plr, tags=[:TERRAIN,:MATCH])


  nothing
end


##

"""
    PartialLinearRelative

IIF will deal with partials, 

DevNotes
- FIXME ongoing (2021Q2), see #1206, likely also #1010
"""
struct PartialLinearRelative{B <: IIF.SamplableBelief, T <: Tuple} <: IIF.AbstractRelativeMinimize
  Z::B
  partial::T # which dimension the partial applies to 
end

function IIF.getSample(s::CalcFactor{<:PartialLinearRelative}, N::Int=1)
  return ( reshape(rand(s.factor.Z,N),1,N), )
end

(s::CalcFactor{<:PartialLinearRelative})(z, x1, x2) = z .- (x2 .- x1)

## build the poses

fg = initfg()
NS = 15

addVariable!(fg, :x0, Point2, tags=[:POSE;])
addFactor!(fg, [:x0;], PriorPoint2(MvNormal([0;0.0], [0.01;0.01])))

driveOneBox!(fg, nothing, start=0.0, NS=NS)



##

# plr = PartialLinearRelative(bel, (1,))
# plr = PartialLinearRelative(bel, (1,))

driveOneBox!(fg, nothing, start=NS, NS=NS)


bel = Normal(10,1.0)
bel = AliasingScalarSampler(..., ...)

plr = PartialLinearRelative(bel, (1,))




##

# driveOneBox!(fg, nothing, start=2*NS, NS=NS)




##


#