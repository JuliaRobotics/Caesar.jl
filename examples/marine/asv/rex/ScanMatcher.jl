"""
this script fetches sequential pairs of poses, fetches the big data (radar pings) tied to those poses, and then determines the pairwise factors that should be added between these sequential pairs
"""

using GraphPlot
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

# fetch variables containing a full sweep
allSweepVariables = filter(v -> :RADARSWEEP in getBigDataKeys(v), getVariables(fg)) |> sortDFG
fsvars = allSweepVariables .|> getLabel

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
sweeps = map(v -> fetchSweep(getVariable(fg, v), datastore), fsvars)
using Images, ImageView
# Filter the images
kg = Kernel.gaussian(7)
sweeps = map(s -> imfilter(s, kg), sweeps)
# Normalize
sweeps = map(s -> s/maximum(s), sweeps)

# Clamp out NaN's:
#map(clamp01nan, img)
#
# using ImageMagick
# for i in 5:length(sweeps)
#   s = sweeps[i]./maximum(sweeps[i])
#   save("/home/gearsad/SlamInDb/seagrant/$i.jpg", Gray.(s))
# end

# At this point we can load the sweeps; let's work on registration
# First step is to have a function that evaluates the cost of a given transform
# between two subsequent images.

using CoordinateTransformations
using ImageTransformations

# this function uses the sum of squared differences between the two images.
# To use low-passed versions of the images, simpy set the kernel argument to
# Kernel.gaussian(10) (or an appropriate size)
function getMismatch(a,b)
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

## Building the graph
using DocStringExtensions
include("RadarFactor.jl")

using LinearAlgebra
using Optim

startsweep = 5
endsweep = 10
graphinit = false

# newfg = initfg()
newfg = generateCanonicalFG_ZeroPose(varType=Pose2)
for i in 1:(endsweep-startsweep)
    addVariable!(newfg, Symbol("x$i"), Pose2, solvable=1)
end
for i in 1:(endsweep-startsweep)
    factor = AlignRadarPose2(sweeps[i+startsweep-1], sweeps[i+startsweep], MvNormal(zeros(3), diagm([5;5;pi/4])))
    addFactor!(newfg, Symbol.(["x$(i-1)", "x$i"]), factor, graphinit=graphinit, solvable=1)
end

# Run the initialization (very slow right now)
# ensureAllInitialized!(newfg)

# Factor debugging
# fs = getFactorFunction.(getFactor.(newfg, lsf(newfg)))
# fs = filter(f -> f isa AlignRadarPose2, fs)
# pf = convert.(PackedAlignRadarPose3, fs)
# convert.(AlignRadarPose2, pf)

# Save the graph
saveDFG(newfg, "$dfgDataFolder/segment_test.tar.gz");

lsf(newfg)
# this should run the radar alignment
pts = approxConv(newfg, :x0x1f1, :x1)

# solving will internally call ensureAllInitialized!(newfg)
tree, smt, hist = solveTree!(newfg)

## Looking at the results
using Plots

ppes = map(v -> getSuggestedPPE(getPPE(getVariable(newfg, v))), ls(newfg))
x = map(ppe -> ppe[1], ppes); y = map(ppe -> ppe[2], ppes); h = map(ppe -> ppe[3], ppes)
Plots.plot(x, y, title="Path Plot", lw=3)


## Stuff
using Optim


cost(x, im1, im2) = evaluateTransform(im1,im2,x[1],x[2],x[3])


# Plotting
xrange = -100.0:1.0:100.0
hrange = -pi:0.1:pi
val = reshape(
    [sweepx(sweeps[10],sweeps[11],xrange); sweepx(sweep_original[10],sweep_original[11],xrange)],
    length(xrange), 2)
Plots.plot(xrange,val)
# Heading
val = reshape(
    [sweeph(sweeps[10],sweeps[11],hrange); sweeph(sweep_original[10],sweep_original[11],hrange)],
    length(hrange), 2)
Plots.plot(hrange,val)

corr_func = (a,b)->sqrt(sum((a .- 0.5).*(b .- 0.5)))
val = reshape(
    [sweepx(sweeps[10],sweeps[11],xrange,diff_func=corr_func);
    sweepx(sweep_original[10],sweep_original[11],xrange,diff_func=corr_func)],
    length(xrange), 2)
Plots.plot(xrange,val)

## Sweep plotting
# sanity check: identity transform should yield zero cost
# @assert evaluateTransform(sweeps[11],sweeps[11],0.,0.,0.) == 0 "There's error with no transform!"

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
