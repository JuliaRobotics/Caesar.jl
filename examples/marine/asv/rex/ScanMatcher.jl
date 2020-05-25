"""
this script fetches sequential pairs of poses, fetches the big data (radar pings) tied to those poses, and then determines the pairwise factors that should be added between these sequential pairs
"""
using DistributedFactorGraphs
using IncrementalInference, RoME
using JSON2

# Where to fetch data
dfgDataFolder = joinpath(ENV["HOME"],"data","rex")
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

# show a couple of sweeps just to test
using Images, ImageView
imshow(sweeps[10])
imshow(sweeps[11])


# At this point we can load the sweeps; let's work on registration
# First step is to have a function that evaluates the cost of a given transform
# between two subsequent images.

using CoordinateTransformations
using ImageTransformations

# this function uses the sum of squared differences between the two images.
# To use low-passed versions of the images, simpy set the kernel argument to
# Kernel.gaussian(10) (or an appropriate size)
function getMismatch(a,b; kernel=[1])
    sqrt(sum((imfilter(a, kernel).-imfilter(b, kernel)).^2))
end

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
evaluateTransform(sweeps[11],sweeps[11],1.,0.,0.)

# let's try small displacements:
sweepx(im1, im2, xrange) = (x->@show evaluateTransform(im1,im2,x,0.,0.)).(xrange)
sweepy(im1, im2, yrange) = (y->@show evaluateTransform(im1,im2,0.,y,0.)).(yrange)
sweeph(im1, im2, hrange) = (h->@show evaluateTransform(im1,im2,0.,0.,h)).(hrange)


using Plots
xrange = -10:0.1:10
hrange = -pi:0.1:pi
Plots.plot(xrange,sweepx(sweeps[10],sweeps[10],xrange))
Plots.plot(xrange,sweepy(sweeps[10],sweeps[10],xrange))
Plots.plot(hrange,sweeph(sweeps[10],sweeps[11],hrange))
