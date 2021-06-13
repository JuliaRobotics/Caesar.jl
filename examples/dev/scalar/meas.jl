#=
Simple Bayesian estimation example on 1D terrain-relative localization.
=#

using DataInterpolations
using Distributions  # measurement models
using ImageFiltering # conv (same), see https://discourse.julialang.org/t/convolution-conv-with-same-size-output/38260/10
using Plots          # uhh, let me be clear
gr()

## 0. Terrain gen
function randu(low::Float64, high::Float64)::Float64
    # rand will return an array, so need to index into it
    return low + (high-low)*rand(Float64,1)[1]
end

function mpe(h)
    # midpoint elevation algorithm
    he = zeros(2*length(h)-1)
    he[1:2:end] = h   # original values
    # midpoints
    for i in 2:2:(length(he)-1)
        (low, high) = minmax(he[i-1],he[i+1])
        he[i] = randu(low,high)
    end
    return he
end

terrain = 4*randn(10)
for i in 1:8
  terrain= mpe(terrain)
end

x = range(-20, 20,length=length(terrain))
plot(x,terrain,label="height")

# interpolated version 
terrain_ = LinearInterpolation(terrain, x)


## 1. Prior and motion model
function convs(signal, kernel)
    # convenience wrapper to get same length on output signal
    return imfilter(signal, reflect(centered(kernel)), Fill(0) )
end

function propagate(prior, model)
    # propagate the system prior through the motion model
    convs(prior, model)
end

function neval(dist, x)
    d = pdf.(dist, x)
    d./(diff(x)[1]*sum(d))
end

motion_model =  Normal(1,0.1)
motion_kernel = neval(motion_model, x)
plot(x, motion_kernel, label="kernel")
# prior distribution on position
prior = neval(Normal(-5,0.1), x)
dx = diff(x)[1]
plot!(x, prior./(dx*sum(prior)),label="prior")

post = propagate(prior, motion_kernel)
plot!(x, post./(dx*sum(post)), label="post")

for i=1:3
    post = propagate(post, motion_kernel)
    plot!(x, post./(dx*sum(post)), label="post")
end


## 2. measurement model
function measurementLikelihood( xt,
                                terrain::LinearInterpolation)
  #
  y = terrain(xt)
    # y = terrain[argmin(abs(xt .- x))]
  alti = Normal(y, 1) # meas model
  p = pdf.(alti, terrain.u)
  return p ./ sum(p)
end



terrain_(-5.1) # if terrain is DataInterpolations.jl object, you can do terrain(xt)

pxz = measurementLikelihood( -5, terrain_ )
  # # terrain height at the true position
  # y = terrain[argmax(prior)] # true measurement
  # # measurement model
  # alti = Normal(y, 1) # measurement + uncertaint
  # # compute p(x|z) and plot normalized
  # pxz = pdf.(alti, terrain)
  # pxz./(dx*sum(pxz))

plot(x, pxz, label="likelihood")


## 3. sim
post = neval(Normal(-5,0.1),x) # est position vs. xt
xt= -5.0
for i=1:10
    # true val (true position), like in dirac postion point
    xt= xt+1.0

    # propagate
    post = propagate(post, motion_kernel)

    # measurement
    likelihood = measurementLikelihood(xt, terrain_)
      # y = terrain[argmin(abs(xt-x))]
      # alti = Normal(y, 1) # meas model
      # likelihood = pdf.(alti, terrain)

    # update
    post = post.*likelihood
end 


## implements a scalar measurement factor (e.g. elevation against known scalar map)
# addFactor!(fg, [:x1, :topography], ScalarFieldFactor(y))
struct ScalarFieldFactor{T<:SamplableBelief} <: IIF.AbstractRelative__
  altimeter::T
end

function IIF.getSample(s::CalcFactor{<:ScalarFieldFactor}, N::Int=1)
    # 1. generate measurement model - 
    #   1a. (high-level): we get a single value (e.g. 20m elevation) - Normal(y, sigma)
    #   1b. (low-level):  we get a vector-valued measurement (e.g. back-scatter strength) - Normal(argmax(y),sigma) [classic] or BSS(y) [full measurement]
    #   NOTE: 1b is not the n-dimensional vector field scenario; it is just a more complicated (un-processed) measurement
    # 2. generate likelihood (eval measurement model over field) 
    # 3. draw samples from likelihood

  return (reshape(rand(s.altimeter,N),1,N),)
end

# keep track of IIF #1051
function (s::CalcFactor{<:ScalarFieldFactor})(elevMeasurement, pose_i)
  # assume pose_i is from a Point2
  # magically need access to global constant fields already loaded
  # addConstant!(fg, :topography, ScalarField(....,manifoldDomain))

  # NOTE: topoMap must support 2d interpolation to allow for `topoMap(pose)` querying 
  topoMap = s.constants[:topography] # still early estimate of API
  
  
  # terrain_ = LinearInterpolation(terrain, x)
  # x,y is test point on prior data
  residual = elevMeasurement - topoMap(pose_i) # topoMap == terrain_
  
  return residual
end

##

#=
# Below: discussion on syntax for different use cases

## for 1a.
# user only identifies the altimeter reading (map magically happens)
# pxz = measurementLikelihood(); 
# likelihoodSampler = bss(x, pxz)
# addFactor!(fg, :x6, Point1Prior(bss))

# semantic question on addConstant!(fg,...) vs. addField!(fg, ...)
#  con field is overloaded term
#  con constant doesnt at first caputer

fg = initfg()
# addField!()
addConstant!(fg, :m, ScalarField(......????)) # load DEM here # on what manifold?

addVariable!(fg, :x0, ContinuousEuclid{1})
addVariable!(fg, :x1, ContinuousEuclid{1})
addFactor!(fg, [:x0;:x1], LinearRelative(Normal(1, 0.1)))
addVariable!(fg, :x2, ContinuousEuclid{1})
addFactor!(fg, [:x1;:x2], LinearRelative(Normal(1, 0.1)))
addVariable!(fg, :x3, ContinuousEuclid{1})
addFactor!(fg, [:x2;:x3], LinearRelative(Normal(1, 0.1)))
addVariable!(fg, :x4, ContinuousEuclid{1})
addFactor!(fg, [:x3;:x4], LinearRelative(Normal(1, 0.1)))
altiMeasure = 10
addFactor!(fg, [:x3, :m], Scalar(Normal(altiMeasure, 0.1)))

# localizaton
addConstant!(fg,  :m, ScalarField(...))
addFactor!(fg, [:x1, :m], ScalarMeasurement(Normal(), ))

addConstant!(fg, :wmm, VectorField( "nasa_wmm_here"))
# wmm: V = f(x,y,z)

## DF mental blocker on `Variable` aspect here
addVariable!(fg, :wmm, SpatialVectorField( )) # for bar magnet
addFactor!(fg, [:x7, :wmm], VectorMeas(y_mag))
# mapping
addVariable!(fg, :m, BandLimitedScalarField(.....))
addVariable!(fg, :m, HistogramScalarField(.....))
addVariable!(fg, :m, GriddedScalarField(.....))
##

# variation: static vs dynamic
#  - static does not convey as much as addConstant

# addFactor -> addSampling, addSampler, ...

# addVariable!
# addFactor!
# addConstant?
#  - value varies of space and time


# addVariation good and should be in API for e.g. calibration, etc. reasons.
#  - ambiguous time vs. space varying aspect
#  - whos the target audience of the API naming
#  - too close to sensitivity/perturbation analysis lingo
#  - VicPrk eg, addVariation! (... wheelbase ... ), addVariation!( ... wheeldiameter ... )

#
# for 1b
# (x,h) samples on band-limited terrain
# addFactor!(fg, :x6, :m, ScalarBeam(h,b))


## to get to factor graph
#=
1. 1D localization 
  a.  Unaries (known map) Keep map in fg node somehow (gaps/duty-cycle measurements)
  b.  mapping -- terrain as a variable to be estimated; constraints are now 
      pairwise (map no longer encapsulated in unaries)
2. 1D via gaps -> 2D no map (line intersect)-> 2D point intersects only
  a. beam pattern somewhere in here too.

1
=#
##
