
# The plan, duh duh duuuuh!... 
# - close out boxy with early results?
# - Start down path of exper 5, which is circular, w/ low res prior (?)




##

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
using JSON2
# using ImageView

Gadfly.set_default_plot_size(35cm,25cm)

prjPath = dirname(dirname(pathof(Caesar)))

# this includes Sequences module
include( joinpath(prjPath, "examples","dev","scalar","Sequences.jl") )

## Case 4.
# Square helix northward, with overlapping east/west segments - mapping
# no prior map

# load dem (18x18km span)
img = load(joinpath(prjPath, "examples","dev","scalar","dem.png")) .|> Gray
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


##

function driveLeg!( fg, 
                    startingPose, 
                    start,                    
                    v,
                    terr,
                    direction::Symbol)
  # fg - factor graph object
  # startingPose
  # start - 
  # v - displacement vector (e.g. [NS; 0.0] for direction= :north)
  # terr - terrain vector (1D interpolant object)
  # direction - direction symbol (:north,:south,:east,:west)

  # add end pose of the leg
  newPose = nextPose(startingPose)
  addVariable!(fg, newPose, Point2, tags=[:POSE,direction])

  addFactor!(fg, [newPose;], PartialPrior(Normal(0.1, 0.1),(2,)))

  p2p = Point2Point2(MvNormal(v, [1.0; 1.0]))
  addFactor!(fg, [startingPose; newPose], p2p, tags=[:ODOMETRY; direction])

  # generate odometry and measurement data for this leg
  # avert your eyes: this assumes v is axis-aligned
  # To pay this technical debt: generate a 2D linspace and use it to query a 2D DEM interpolant
  x_seq = LinRange(start, start + maximum(v), 100) # assumed 100 measurements per leg

  z_seq = terr.(x_seq)
  # add noise to x_seq (odometry) and z_seq (meas noise)
  x_seq_n = 0.01*randn(100)
  z_seq_n = 1.0*randn(100)
  
  someDict = Dict(:x_seq => x_seq, :z_seq => z_seq, :x_seq_n => x_seq_n, :z_seq_n => z_seq_n)
  addData!(fg, :default_folder_store, newPose, :elevationSequences, Vector{UInt8}(JSON2.write( someDict )), mimeType="application/json/octet-stream"  )
  
  return newPose
end
# function driveLeg(x0, terrain, v, units)
#   x = zeros(2,0)
#   xm = zeros(2,0)
#   z = zeros(0)
#   zm = zeros(0)

#   for i=1:units
#       x.append(x[end]+v)
#       xm.append(x[end]+ randn(2,1)) # noisy position
      
#       z.append( terrain(x[end]))  # not quite as it isn't 2d interp
#       zm.append(z + sigma_z*randn(1))
#   end

#   # return pose, measurements (valid measured, )
# end


# matchLeg!(fg, [:x1, :x5], :NORTH)
# matchLeg!(fg, [:x3, :x7], :SOUTH)
function matchLeg!( fg::AbstractDFG,
                    legs::AbstractVector{Symbol}, 
                    direction::Symbol;
                    dofactor::Bool=false)
  # 
  # ls(fg, tags=[:NORTH])

  dataEntry, dataBytes = getData(fg, legs[1], :elevationSequences)
  myData_a = JSON2.read(IOBuffer(dataBytes), Dict{Symbol, Vector{Float64}})

  dataEntry, dataBytes = getData(fg, legs[2], :elevationSequences)
  myData_b = JSON2.read(IOBuffer(dataBytes), Dict{Symbol, Vector{Float64}})


  xseq_a = myData_a[:x_seq]
  zseq_a = myData_a[:z_seq]

  xseq_b = myData_b[:x_seq]
  zseq_b = myData_b[:z_seq]

  # correlate z_seq0 against z_seq_m4

  s_a  = Sequences.MeasurementSequence(xseq_a, zseq_a)
  s_b = Sequences.MeasurementSequence(xseq_b.-xseq_b[1], zseq_b)
  # FIXME ON FIRE dont have odo ground truth
  # see also: [x,i] = ssdcorr((x,z)_a, (x,z)_b)

  # closure on a unique correlator with s_a and s_b
  _ssdCorr(x) = Sequences.ssd(s_a, Sequences.displace(s_b,x))

  qr = collect(LinRange(-10.0,10.0,10001)) #range over which to compute ssd
  intensity = _ssdCorr.(qr)
  k = abs(maximum(intensity))
  pv = intensity./k
  pv = exp.(-pv)
  pv ./= sum(pv)

  # intensity = _mySSDCorr(zseq_a, zseq_b)

  bel = AliasingScalarSampler(qr, pv)
  plr = PartialLinearRelative(bel, (1,))

  f_ = if dofactor
    addFactor!(fg, legs, plr, tags=[:TERRAIN,:MATCH,direction])
  end

  return (qr, pv), f_
end


##

nextPose(ps::Symbol; pattern=r"x") = Symbol("x",match(r"\d+", string(ps)).match |> x->(parse(Int,x)+1))

# drive clockwise, x is North, y is East (NED convention).
# boxes start bottom left, spine of boxy helix is on x-axis
function driveOneBox!(fg;
                      lastPose=sortDFG(ls(fg, tags=[:POSE]))[end],
                      start = [0.0;0.0],
                      runback = 2/3,
                      NS = 15,
                      EW = 15,
                      docorr::Bool=false  )
  #
  global terrE, terrW
  #
  img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples/dev/scalar/dem.png")) .|> Gray
  h = 1e3*Float64.( @view img[512,:])
  start_x, start_y = (start...,)
  
  # drive North NS units (northbound uses west slice)
  lastPose = driveLeg!(fg, lastPose, start_x, [NS;0.0] ,terrW, :NORTH)
  start_x = start_x+NS

  # drive East EW units (terrain does not matter here)
  lastPose = driveLeg!(fg, lastPose, start_y ,[0.0; EW], terrW, :EAST)
  start_y = start_y + EW
  
  # drive South 0.7NS units
  lastPose = driveLeg!(fg, lastPose, start_x, [-runback*NS;0.0],terrE, :SOUTH)
  start_x = start_x - runback*NS

  # drive West EW units (terrain does not matter here)
  lastPose = driveLeg!(fg, lastPose, start_y, [0.0; -EW], terrE, :WEST)
  start_y = start_y - EW

  # cop-out early on first run, or debug
  # docorr ? nothing : (return nothing)

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

# trajectory parameters
NS = 15
runback = 2/3

# setup the factor graph
fg = initfg()

# adding a blog store
# getSolverParams(fg).logpath = pwd()
storeDir = joinLogPath(fg,"data")
mkpath(storeDir)
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg, datastore)


##

# actually start adding nodes 
addVariable!(fg, :x0, Point2, tags=[:POSE;])
addFactor!(fg, [:x0;], PriorPoint2(MvNormal([0;0.0], [0.01;0.01])))


# drive first box, no correlations
driveOneBox!(fg, runback=runback, start=[0.0;0], NS=NS, docorr=false)

# plr = PartialLinearRelative(bel, (1,))
# plr = PartialLinearRelative(bel, (1,))

# drive second box, implement correlation externally
driveOneBox!(fg, runback=runback, start=[(1-runback)*NS;0], NS=NS, docorr=false)


##

xsq, f_ = matchLeg!(fg, [:x1, :x5],:NORTH, dofactor=false)

xsq, f_ = matchLeg!(fg, [:x3, :x7],:SOUTH, dofactor=false)

## ## TEMPORARY CORRELATION DEV

Gadfly.plot(x=xsq[1], y=xsq[2], Geom.line)




# addFactor!(fg, legs, plr, tags=[:TERRAIN, :MATCH])

##

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






## ## TEMPORARY


##




bel = Normal(10,1.0)
bel = AliasingScalarSampler(..., ...)

plr = PartialLinearRelative(bel, (1,))




##




##


#