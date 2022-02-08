
#= 
Proof of concept of 2D localization against a scalar field.
In this example we use a sequence of 1D measurements and a known map with a
correlator-like approach to localize.

Case 4: Square helix northward, with overlapping east/west segments 
no prior map

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
st=style( major_label_font="CMU Serif",
          minor_label_font="CMU Serif",
          major_label_font_size=20pt,
          minor_label_font_size=20pt)


prjPath = pkgdir(Caesar)
# this includes Sequences module
include( joinpath(prjPath, "examples","dev","scalar","Sequences.jl") )


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


"""
    Point2Point2Northing

IIF will deal with partials, 

DevNotes
- FIXME ongoing (2021Q2), see #1206, likely also #1010
"""
struct Point2Point2Northing{B <: IIF.SamplableBelief, T <: Tuple} <: IIF.AbstractRelativeMinimize
  Z::B
  partial::T # which dimension the partial applies to 
end

function IIF.getSample(s::CalcFactor{<:Point2Point2Northing}, N::Int=1)
  return ( reshape(rand(s.factor.Z,N),1,N), )
end

(s::CalcFactor{<:Point2Point2Northing})(z, x1, x2) = z .- (x2[1] - x1[1])


##

# matchLeg!(fg, [:x1, :x5], :POSITIVE_X)
# matchLeg!(fg, [:x3, :x7], :NEGATIVE_X)
function matchLeg!( fg::AbstractDFG,
                    legs::AbstractVector{Symbol}, 
                    direction::Symbol;
                    dofactor::Bool=false,
                    odoPredictedAlign::Real=0,
                    kappa::Real=2)
  # 

  dataEntry, dataBytes = getData(fg, legs[1], :elevationSequences)
  myData_a = JSON2.read(IOBuffer(dataBytes), Dict{Symbol, Vector{Float64}})

  dataEntry, dataBytes = getData(fg, legs[2], :elevationSequences)
  myData_b = JSON2.read(IOBuffer(dataBytes), Dict{Symbol, Vector{Float64}})


  xseq_a = myData_a[:x_seq]
  zseq_a = myData_a[:z_seq]

  xseq_b = myData_b[:x_seq]
  zseq_b = myData_b[:z_seq]

  # correlate z_seq0 against z_seq_m4
  s_a = Sequences.MeasurementSequence(xseq_a .- xseq_a[1], zseq_a) # full relative
  s_b = Sequences.MeasurementSequence(xseq_b .- xseq_b[1] .- odoPredictedAlign, zseq_b)
  # FIXME ON FIRE dont have odo ground truth
  # see also: [x,i] = ssdcorr((x,z)_a, (x,z)_b)

  # closure on a unique correlator with s_a and s_b
  _ssdCorr(x) = Sequences.ssd(s_a, Sequences.displace(s_b,x))

  # qr is the range of displacements over which to compute the  SSD
  qr = collect(LinRange(-10.0,10.0,1001)) 
  intensity = _ssdCorr.(qr)
  # @assert length(intensity) == length(qr)
  # strip nans when no overlap
  mask::BitVector = isnan.(intensity) # .|> x->xor(x, true)
  intensity[mask] .= maximum(intensity[xor.(mask, true)]) #intensity[mask]
  k = abs(maximum(intensity))
  pv = intensity./k  # normalize
  pv = exp.(-pv)
  w = DSP.Windows.tukey(length(intensity), 0.25)
  pv .*= w
  pv .^= kappa
  # hand fix, to emphasize peaks

  # pv ./= sum(pv) # sum to 1, already in AliasingScalarSampler

  # intensity = _mySSDCorr(zseq_a, zseq_b)

  bel = AliasingScalarSampler(qr, pv)
  plr = Point2Point2Northing(bel, (1,))

  f_ = if dofactor
    addFactor!(fg, legs, plr, tags=[:TERRAIN,:MATCH,direction])
  end

  return (qr, pv), f_
end



##



# callback to structure sequences
function newcallback_ex4(fg::AbstractDFG, lastPose::Symbol)
  # temp workaround for terrain interpolators
  
  @show lastPose
  @show tags = getVariable(fg, lastPose).tags
  @show prevPose = incrSuffix(lastPose, -1)
  
  # don't do anything extra on first pose :x0
  if lastPose == :x0
    return nothing
  end
  
  # true location of poses in world
  simPos = getPPE(fg, lastPose, :simulated).suggested
  # generate xseq and zseq using this and previous pose data
  simPosPrev = getPPE(fg, prevPose, :simulated).suggested
  # linear for Point2Point2
  @show simOdo = simPos - simPosPrev
  
  # add a partial prior in Y if no full prior
  if 0 == length( intersect(ls(fg, lastPose), lsf(fg, PriorPoint2)) )
    pp = PartialPrior(Normal(simPos[2], 0.1),(2,))
    @info "adding partial prior on simulated y position" lastPose simPos[2] 
    addFactor!(fg, [lastPose;], pp, graphinit=false, tags=[:EX4_PARTIALPRIOR_Y])
  end
  
  
  # assumed 100 measurements per leg
  Nseq = 100
  
  # avert your eyes: this assumes v is axis-aligned
  # To pay this technical debt: generate a 2D linspace and use it to query a 2D DEM interpolant
  terr, x_seq = if abs(simOdo[2]) < abs(simOdo[1])
    global terrW
    # assume north south
    xs = LinRange(simPos[1], simPosPrev[1], Nseq)
    terrW, xs
  elseif abs(simOdo[1]) < abs(simOdo[2])
    global terrE
    # assume east west
    xs = LinRange(simPos[2], simPosPrev[2], Nseq)
    terrE, xs
  end

  z_seq = terr.(x_seq)
  # add noise to x_seq (odometry) and z_seq (meas noise)
  x_seq_n = 0.01*randn(Nseq)
  z_seq_n = 1.0*randn(Nseq)
  
  @assert !isapprox( Statistics.median(diff(x_seq)), 0, atol=1e-6) "why is x_seq[1:5]=$(x_seq[1:5])"

  someDict = Dict(:x_seq => x_seq, :z_seq => z_seq, :x_seq_n => x_seq_n, :z_seq_n => z_seq_n)
  addData!( fg, :default_folder_store, lastPose, :elevationSequences, 
            Vector{UInt8}(JSON2.write( someDict )), 
            mimeType="application/json/octet-stream"  )
  #

  nothing
end




##

# trajectory parameters
NS = 15
skew_x = 2/3

# setup the factor graph
fg = initfg()

# adding a blog store
# getSolverParams(fg).logpath = pwd()
storeDir = joinLogPath(fg,"data")
mkpath(storeDir)
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg, datastore)



##

# drive the first box, no correlations
RoME.generateCanonicalFG_Boxes2D!(8, dfg=fg, skew_x=skew_x, postpose_cb=newcallback_ex4)



##

# 1st match: north-bound legs on first and second loop
xsq, f_ = matchLeg!(fg, [:x0, :x4], :POSITIVE_X, odoPredictedAlign=0, kappa=3, dofactor=false);
# xsq, f_ = matchLeg!(fg, [:x1, :x5],:NORTH, odoPredictedAlign=0, kappa=3, dofactor=true)
# p=Gadfly.plot(st)
# push!(p,layer(x=xsq[1], y=xsq[2], Geom.line))

# 2nd match: south-bound legs on first and second loop
xsq, f_ = matchLeg!(fg, [:x2, :x6], :NEGATIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true);



# 1st match: north-bound legs on first and second loop
xsq, f_ = matchLeg!(fg, [:x4, :x8],:POSITIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true);
# xsq, f_ = matchLeg!(fg, [:x1, :x5],:POSITIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true)
# p=Gadfly.plot(st)
# push!(p,layer(x=xsq[1], y=xsq[2], Geom.line))

# 2nd match: south-bound legs on first and second loop
xsq, f_ = matchLeg!(fg, [:x6, :x10],:NEGATIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true);


xsq, f_ = matchLeg!(fg, [:x8, :x12],  :POSITIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true);
xsq, f_ = matchLeg!(fg, [:x10, :x14], :NEGATIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true);

# xsq, f_ = matchLeg!(fg, [:x12, :x16],  :POSITIVE_X, odoPredictedAlign=0, kappa=3, dofactor=true)

##

pts = approxConv(fg, :x0x4f1, :x4)

# should peak at +5 to make the relative factor represent the change between x0 dn x4
Gadfly.plot(x=pts[1,:], Geom.histogram)

## TEMPORARY CORRELATION DEV


tree = solveTree!(fg)


##


plotKDE(fg, [:x0;:x1;:x2;:x3;:x4], levels=1)
plotKDE(fg, [:x4;:x5;:x6;:x7;:x8], levels=1)

plotKDE(fg, [:x8;:x9;:x10;:x11;:x12], levels=1)
plotKDE(fg, [:x12;:x13;:x14;:x15;:x16], levels=1)


##

# get pose four poses back
# get terrain sequence between (newPose - NW) up to (newPose)
# curr_seq = terrE(curr_x_seq)
# prev_seq = terrE(prev_x_seq)
# # compute SSDcorr wrt sequence 4 poses before
# # add Point2Point2Northing between the two 
# intensity = _mySSDCorr(curr_seq, prev_seq)

plr = ScalarFieldSequenceFactorNorthSouth(x_seq_, z_seq, (1,))
addFactor!(fg, [:x0; :x4], plr, tags=[:TERRAIN,:MATCH])
    # plr = ScalarFieldSequenceFactorNorthSouth(x_seq, z_seq, (1,))
    # # # plr = Point2Point2Northing(grid, AliasingScalarSampler(intensity))
    # addFactor!(fg, [prevPose newPose], plr, tags=[:TERRAIN,:MATCH])







##
