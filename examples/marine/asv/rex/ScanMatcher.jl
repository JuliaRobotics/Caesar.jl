"""
this script fetches sequential pairs of poses, fetches the big data (radar pings) tied to those poses, and then determines the pairwise factors that should be added between these sequential pairs
"""

using DistributedFactorGraphs
using IncrementalInference, RoME
using JSON2

# Where to fetch data
dfgDataFolder = joinpath(ENV["HOME"],"data","seagrant","rex")
datastore = FileDataStore("$dfgDataFolder/bigdata")
# Load the graph
fg = initfg()

# Reformat the data.
loadDFG!(fg, "$dfgDataFolder/fullsweep_updated.tar.gz");

# disable solving for all variables
map(s->setSolvable!(fg, s, 0), ls(fg))

# fetch variables containing a full sweep
allSweepVariables = filter(v -> :RADARSWEEP in getBigDataKeys(v), getVariables(fg));
# make only those variables solvable
fsvars = allSweepVariables .|> getLabel
map(s->setSolvable!(fg, s, 1), fsvars)

# fetch and sort solvable variables
activeVars = ls(fg, solvable=1) |> sortDFG

# helper function to retrieve the radar sweep for a given variable
function fetchSweep(var::DFGVariable, store::FileDataStore)
    entry = getBigDataEntry(var, :RADARSWEEP)
    rawData = getBigData(datastore, entry)
    # raw data is json-encoded; this decoding should happen inside getBigData?
    rawdata = Vector{Float64}(JSON2.read(IOBuffer(rawData)))
    n = Int(sqrt(length(rawdata)))
    sweep = reshape(rawdata,(n,n))
    return sweep # That's pretty sweep if i say so myself...
end

# fetch all radar pings
sweeps = map(v -> fetchSweep(getVariable(fg, v), datastore), activeVars)
# fs10 = imfilter(sweeps[10],Kernel.gaussian(7))


# show a couple of sweeps just to test
using Images, ImageView
imshow(sweeps[10])
imshow(sweeps[11])


# imshow(imfilter(sweeps[10],Kernel.gaussian(15)) )


# At this point we can load the sweeps; let's work on registration
# First step is to have a function that evaluates the cost of a given transform
# between two subsequent images.

using CoordinateTransformations
using ImageTransformations

# this function uses the sum of squared differences between the two images.
# To use low-passed versions of the images, simpy set the kernel argument to
# Kernel.gaussian(10) (or an appropriate size)
function getMismatch(a,b; kernel=[1])
    sqrt(sum((a.-b).^2))
end
# sqrt(sum( imfilter( a.-b, Kernel.gaussian(5)).^2 ))
# sqrt(sum((imfilter(a, kernel).-imfilter(b, kernel)).^2))

# also do MMD version


# Next step is to define a function that applies a transform to the image. This
# transform consists of a translation and a rotation
function transformImage(img::Array{Float64,2}, dx::Real, dy::Real, dh::Real)
    tf = LinearMap(RotMatrix(dh))∘Translation(dx,dy)
    tf_img = warp(img, tf)

    # replace NaN w/ 0
    mask = findall(x->isnan(x),tf_img)
    tf_img[mask] .= 0.0
    return tf_img
end


# now we can combine the two into an evaluation function
function evaluateTransform(a::Array{Float64,2},b::Array{Float64,2}, dx::Float64, dy::Float64, dh::Float64)
    # transform image
    bp = transformImage(b,dx,dy,dh)
    # get matching padded views
    ap, bpp = paddedviews(0.0, a, bp)
    return getMismatch(ap,bpp)
end

# sanity check: identity transform should yield zero cost
@assert evaluateTransform(sweeps[11],sweeps[11],0.,0.,0.) == 0 "There's error with no transform!"

# transform image
bp = transformImage(sweeps[11],0,0,0)
# get matching padded views
ap, bpp = paddedviews(0.0, sweeps[11], bp)
sum(ap.-bpp)

# let's try small displacements:
# sweepx(im1, im2, xrange) = (x->@show evaluateTransform(im1,im2,x,0.,0.)).(xrange)
# sweepy(im1, im2, yrange) = (y->@show evaluateTransform(im1,im2,0.,y,0.)).(yrange)
# sweeph(im1, im2, hrange) = (h->@show evaluateTransform(im1,im2,0.,0.,h)).(hrange)


# using Plots
# xrange = -10:0.1:10
# hrange = -pi:0.1:pi
# Plots.plot(xrange,sweepx(sweeps[10],sweeps[11],xrange))
# Plots.plot(xrange,sweepy(sweeps[10],sweeps[11],xrange))
# Plots.plot(hrange,sweeph(sweeps[10],sweeps[11],hrange))


# fs10 = imfilter(sweeps[10],Kernel.gaussian(3))
# fs11 = imfilter(sweeps[11],Kernel.gaussian(3))
# ffs10 = imfilter(fs10,Kernel.gaussian(3))
# ffs11 = imfilter(fs11,Kernel.gaussian(3))
#
# Plots.plot(xrange,sweepx(ffs10,ffs11,xrange))
# Plots.plot(xrange,sweepy(fs10,fs11,xrange))



## making our own factors

using TensorCast

import IncrementalInference: getSample
import Base: convert


"""
$TYPEDEF

This is but one incarnation for how radar alignment factor could work, treat it as a starting point.

Example
-------
```julia
using LinearAlgebra
arp2 = AlignRadarPose2(sweep[10], sweep[11], MvNormal(zeros(3), diagm([5;5;pi/4])))
```

"""
struct AlignRadarPose2{T <: Real, P <: SamplableBelief} <: FunctorPairwise
  im1::Matrix{T}
  im2::Matrix{T}
  PreSampler::P
  p2p2::Pose2Pose2
end

AlignRadarPose2(im1::Matrix{T}, im2::Matrix{T}, pres::P) where {T <: Real, P <: SamplableBelief} = AlignRadarPose2{T,P}(im1, im2, pres, Pose2Pose2(pres))

function getSample(rp2::AlignRadarPose2, N::Int=1)

  # closure on inner function
  cost(x::AbstractVector) = evaluateTransform(rp2.im1,rp2.im2,x...)

  pres = rand(rp2.PreSampler, N)
  out = zeros(3, N)
  TASKS = Vector{Task}(undef, N)
  for i in 1:N
    # ignoring failures
      TASKS[i] = Threads.@spawn optimize(cost, view(pres, :, $i), NelderMead()).minimizer
    # out[:,i] = optimize(cost, pres[:,i], NelderMead()).minimizer
  end
    # retrieve threaded results
    @sync for i in 1:N
      @async out[:,$i] .= fetch(TASKS[$i])
    end

  # only using out, but return pres if user wants to look at it.
  return (out, pres)
end

function (rp2::AlignRadarPose2)(res::Vector{Float64},
                           userdata::FactorMetadata,
                           idx::Int,
                           meas::Tuple,
                           wXi::Array{Float64,2},
                           wXj::Array{Float64,2})
  #
  rp2.p2p2(res, userdata, idx, meas, wXi, wXj)
  res
end


struct PackedAlignRadarPose2 <: PackedInferenceType
  im1::Vector{Float64}
  im2::Vector{Float64}
  PreSampler::String
  p2p2::PackedPose2Pose2
end

function convert(::Type{PackedAlignRadarPose2}, arp2::AlignRadarPose2)
  TensorCast.@cast pim1[row][col] := arp2.im1[row,col]
  TensorCast.@cast pim2[row][col] := arp2.im2[row,col]
  PackedAlignedRadarPose2(pim1, pim2, string(arp2.PreSampler), convert(PackedPose2Pose2, arp2.p2p2))
end
function convert(::Type{AlignRadarPose2}, parp2::PackedAlignRadarPose2)
  TensorCast.@cast im1[row,col] := parp2.pim1[row][col]
  TensorCast.@cast im2[row,col] := parp2.pim2[row][col]
  AlignedRadarPose2(im1, im2, extractdistribution(arp2.PreSampler), convert(Pose2Pose2, parp2.p2p2))
end


## Building the graph
using LinearAlgebra
using Optim
numposes = 10
graphinit = false

# newfg = initfg()
newfg = generateCanonicalFG_ZeroPose2()
for i in 1:10
    addVariable!(newfg, Symbol("x$i"), Pose2, solvable=1)
end
for i in 2:10
    factor = AlignRadarPose2(sweeps[i-1], sweeps[i], MvNormal(zeros(3), diagm([5;5;pi/4])))
    addFactor!(newfg, Symbol.(["x$(i-1)", "x$i"]), factor, graphinit=graphinit, solvable=1)
end

# this should run the radar alignment
pts = approxConv(newfg, :x0x1f1, :x1)

# solving will internally call ensureAllInitialized!(newfg)
tree, smt, hist = solveTree!(fg)


## Stuff
using Optim


cost(x, im1, im2) = evaluateTransform(im1,im2,x[1],x[2],x[3])


X0 = zeros(3)
Minimizer: [3.65e-01, -3.50e-01, -2.63e-04]
Minimum:   5.230037e+04


Initial Point: [-1.84e-01, -3.36e-01, -6.51e-01]
Minimizer: [-6.85e-01, 1.85e-01, -3.20e-03]
Minimum:   5.242571e+04


Initial Point: [1.69e-01, 5.36e-01, -1.35e+00]
Minimizer: [-7.24e-01, 2.08e+00, -3.24e-03]
Minimum:   5.221261e+04


Initial Point: [4.89e-01, -1.16e+00, -7.43e-02]
Minimizer: [4.02e-01, -1.35e+00, -2.39e-04]
Minimum:   5.244541e+04
