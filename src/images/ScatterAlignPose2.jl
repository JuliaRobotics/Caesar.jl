

import IncrementalInference: getSample, preambleCache
import Base: convert, show
# import DistributedFactorGraphs: getManifold
import ApproxManifoldProducts: sample

using .Images

export ScatterAlignPose2, PackedScatterAlignPose2

export overlayScatter, overlayScatterMutate

"""
$TYPEDEF

This is but one incarnation for how radar alignment factor could work, treat it as a starting point.

Notes
- Stanard `cvt` argument is lambda function to convert incoming images to user convention of image axes,
  - **Geography map default** `cvt` flips image rows so that Pose2 +xy-axes corresponds to img[-x,+y]
    - i.e. rows down is "North" and columns across from top left corner is "East".
- Use rescale to resize the incoming images for lower resolution (faster) correlations
- Both images passed to the construct must have the same type some matrix of type `T`.

Example
-------
```julia
arp2 = ScatterAlignPose2(img1, img2, 2) # e.g. 2 meters/pixel 
```

See also: [`overlayScanMatcher`](@ref)
"""
Base.@kwdef struct ScatterAlignPose2{ H1 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity},
                                      H2 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity} } <: IIF.AbstractManifoldMinimize
  """ reference image for scan matching. """
  cloud1::H1
  """ test image to scan match against the reference image. """
  cloud2::H2
  """ Common grid scale for both images -- i.e. units/pixel.  
  Constructor uses two arguments `gridlength`*`rescale=1`=`gridscale`.
  Arg 0 < `rescale` ≤ 1 is also used to rescale the images to lower resolution for speed. """
  gridscale::Float64 = 1.0
  """ how many heatmap sampled particles to use for mmd ailgnment """
  sample_count::Int  = 100
  """ bandwidth to use for mmd """
  bw::Float64        = 1.0
end


# replace inner constructor with transform on image
ScatterAlignPose2(im1::AbstractMatrix{T}, 
                  im2::AbstractMatrix{T},
                  domain::Tuple{<:AbstractVector{<:Real},<:AbstractVector{<:Real}};
                  sample_count::Integer=75,
                  bw::Real=5e-5, # from a sensitivity analysis with marine radar data (50 or 100 samples)
                  rescale::Real=1,
                  N::Integer=1000,
                  cvt = (im)->reverse(Images.imresize(im,trunc.(Int, rescale.*size(im))),dims=1)
                ) where {T} = ScatterAlignPose2(;cloud1=HeatmapGridDensity(cvt(im1),domain,N=N), 
                                                cloud2=HeatmapGridDensity(cvt(im2),domain,N=N),
                                                gridscale=float(rescale),
                                                sample_count, 
                                                bw  )
#

getManifold(::IIF.InstanceType{<:ScatterAlignPose2}) = getManifold(Pose2Pose2)

# runs once upon addFactor! and returns object later used as `cache`
function preambleCache(dfg::AbstractDFG, vars::AbstractVector{<:DFGVariable}, fnc::ScatterAlignPose2)
  #
  M = getManifold(Pose2)
  e0 = ProductRepr(SVector(0.0,0.0), SMatrix{2,2}(1.0, 0.0, 0.0, 1.0))

  smps1, = sample(fnc.cloud1, fnc.sample_count)
  smps2, = sample(fnc.cloud2, fnc.sample_count)

  bw = SA[fnc.bw;]
  
  score = Ref(0.0)

  (;M,e0,smps1,smps2,bw,score)
end

Base.@kwdef struct _FastRetract{M_ <: AbstractManifold, T}
  M::M_ = SpecialEuclidean(2)
  pTq::T = ProductRepr(MVector(0,0.0), MMatrix{2,2}(1.0, 0.0, 0.0, 1.0))
  p::ProductRepr{Tuple{SVector{2, Float64}, SMatrix{2, 2, Float64, 4}}} = ProductRepr(SA[0.0;0.0], SMatrix{2,2}(1.0, 0, 0, 1))
end

function (_user::_FastRetract)(pCq::AbstractVector{<:Real})
  retract!(_user.M, _user.pTq, _user.p, hat(_user.M, _user.p, pCq))

  return _user.pTq
end

@doc raw"""
    $SIGNATURES

Transform and put 2D pointcloud as [x,y] coordinates of a-frame into `aP_dest`, by transforming 
incoming b-frame points `bP_src` as [x,y] coords via the transform `aTb` describing the b-to-a (aka a-in-b)
relation.  Return the vector of points `aP_dest`.

````math
{}^a \begin{bmatrix} x, y \end{bmatrix} = {}^a_b \mathbf{T} \, {}^b \begin{bmatrix} x, y \end{bmatrix}
````
"""
function _transformPointCloud2D!(
    # manifold
    M::typeof(SpecialEuclidean(2)),
    # destination points
    aP_dest::AbstractVector,
    # source points
    bP_src::AbstractVector,
    # transform coordinates
    aCb::AbstractVector{<:Real}; 
    # base point on manifold about which to do the transform
    e0 = ProductRepr(SVector(0.0,0.0), SMatrix{2,2}(1.0,0.0,0.0,1.0)),
    backward::Bool=false
  )
  #
  
  aTb = retract(M, e0, hat(M, e0, aCb))
  aTb = backward ? inv(M,aTb) : aTb
  bT_src = ProductRepr(MVector(0.0,0.0), SMatrix{2,2}(1.0,0.0,0.0,1.0))
  aT_dest = ProductRepr(MVector(0.0,0.0), MMatrix{2,2}(1.0,0.0,0.0,1.0))
  
  for (i,bP) in enumerate(bP_src)
    bT_src.parts[1] .= bP
    Manifolds.compose!(M, aT_dest, aTb, bT_src)
    # aP_dest[i][:] = Manifolds.compose(M, aTb, bP_src[i]).parts[1]
    aP_dest[i] .= aT_dest.parts[1]
  end

  aP_dest
end

function _transformPointCloud2D(
    # manifold
    M::typeof(SpecialEuclidean(2)),
    # source points
    bP_src::AbstractVector,
    # transform coordinates
    aCb::AbstractVector{<:Real}; 
    kw...
  )
  #
  #dest points
  aP_dest = typeof(MVector(0.0,0.0))[MVector(0.0,0.0) for _ in 1:length(bP_src)] # Vector{typeof(MVector(0.0,0.0))}(undef, length(bP_src))
  # fill!(aP_dest, MVector(0.0,0.0))
  _transformPointCloud2D!(M, aP_dest, bP_src, aCb; kw...)
end

function getSample( cf::CalcFactor{<:ScatterAlignPose2} )
  #
  M = cf.cache.M
  e0 = cf.cache.e0
  R0 = e0.parts[2]
  
  pVi = cf.cache.smps1
  qVj = cf.cache.smps2
  # Fresh samples
  for i in 1:cf.factor.sample_count
    pVi[i] .= sample(cf.factor.cloud1)[1][1]
    qVj[i] .= sample(cf.factor.cloud2)[1][1]
  end
  
  # qVi(qCp) = _transformPointCloud2D(M,pVi,qCp)
  pVj(qCp) = _transformPointCloud2D(M,qVj,qCp; backward=true)
  
  # bw = SA[cf.factor.bw;]
  cost(xyr) = mmd(M.manifold[1], pVi, pVj(xyr), length(pVi), length(qVj), cf._allowThreads; cf.cache.bw)
  
  # return mmd as residual for minimization
  res = Optim.optimize(cost, [10*randn(2); 0.1*randn()], Optim.BFGS() )
  
  cf.cache.score[] = res.minimum
  
  # give measurement relative to e0 identity
  #  TODO relax to Riemannian where e0 is replaced by any point
  hat(M, e0, res.minimizer)
end


function (cf::CalcFactor{<:ScatterAlignPose2})(pXq, wPp, wPq)
  # 
  
  M = cf.cache.M
  e0 = cf.cache.e0
  
  # get the current relative transform estimate
  wPq_ = Manifolds.compose(M, wPp, exp(M, e0, pXq))
  
  #TODO allocalte for vee! see Manifolds #412, fix for AD
  Xc = zeros(3)
  vee!(M, Xc, wPq, log(M, wPq, wPq_))
  
  return Xc
end


"""
    $SIGNATURES

Overlay the two images from `AlignRadarPose2` with the first (red) fixed and transform the second image (blue) according to `tf`.

Notes:
- `tf` is a Manifolds.jl type `::ProductRepr` (or newer `::ArrayPartition`) to represent a `SpecialEuclidean(2)` manifold point.
- `user_offset` translates both are the same.

See also: [`plotScatterAlign`](@ref)
"""
function overlayScatter(sap::ScatterAlignPose2, 
                        trans::AbstractVector{<:Real}=[0;0.0],
                        rot::Real=0.0;
                        user_coords = [trans; rot],
                        # score=Ref(0.0),
                        sample_count::Integer=sap.sample_count,
                        showscore::Bool=true,
                        findBest::Bool=true  )
  #

  # # sample points from the scatter field
  tfg = initfg()
  # getSolverParams(tfg).attemptGradients = false
  addVariable!(tfg, :x0, Pose2)
  addVariable!(tfg, :x1, Pose2)
  fc = addFactor!(tfg, [:x0;:x1], sap; graphinit=false)

  meas = sampleFactor(tfg, :x0x1f1)[1]

  b_score = IIF._getCCW(tfg, :x0x1f1).dummyCache.score[]

  cac = IIF._getCCW(fc).dummyCache
  PT1 = cac.smps1
  PT2 = cac.smps2

  M = cac.M
  e0 = cac.e0
  R0 = [1 0; 0 1.]

  # not efficient, but okay for here
  # pTq(xyr=user_coords) = exp(M, e0, hat(M, e0, xyr))

  best = vee(M, e0, meas)
  best_coords = round.(best,digits=3)
  
  # transform points1 to frame of 2.  TBD, p as coordinate expansion point?
  pts2T_u = _transformPointCloud2D(M, PT2, user_coords; backward=true)
  pts2T_b = _transformPointCloud2D(M, PT2, best; backward=true)

  @cast __pts1[i,j] := PT1[j][i]
  @cast __pts2[i,j] := PT2[j][i]
  @cast __pts2Tu[i,j] := pts2T_u[j][i]
  @cast __pts2Tb[i,j] := pts2T_b[j][i]
  
  # return NamedTuple obj with all the necessary data for plotting, see plotScatterAlign
  (;pP1=__pts1, qP2=__pts2, pP2_u=__pts2Tu, pP2_b=__pts2Tb, best_coords, user_coords, u_score=round(0,sigdigits=3), b_score=round(b_score[],sigdigits=3)) #, user_offset)
end


function overlayScatterMutate(sap_::ScatterAlignPose2;
                              sample_count::Integer = sap_.sample_count,
                              bw::Real = sap_.bw,
                              kwargs... )
  #
  sap = ScatterAlignPose2(sap_.cloud1, sap_.cloud2, sap_.gridscale, sample_count, float(bw))
  Caesar.overlayScatter(sap; kwargs...);
end


function Base.show(io::IO, sap::ScatterAlignPose2{H1,H2}) where {H1,H2}
  printstyled(io, "ScatterAlignPose2{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    H1 = ", color=:magenta)
  println(io, H1)
  printstyled(io, "    H2 = ", color=:magenta)
  println(io, H2)
  printstyled(io, " }", color=:blue, bold=true)
  println(io)
  if H1 <: HeatmapGridDensity
    println(io, "  size(.data):     ", size(sap.cloud1.data))
    println(io, "    min/max:       ", round(minimum(sap.cloud1.data),digits=3), "/", round(maximum(sap.cloud1.data),digits=3))
  end
  if H2 <: HeatmapGridDensity
    println(io, "  size(.data):     ", size(sap.cloud2.data))
    println(io, "    min/max:       ", round(minimum(sap.cloud2.data),digits=3), "/", round(maximum(sap.cloud2.data),digits=3))
  end

  println(io, "  gridscale:       ", sap.gridscale)
  println(io, "  sample_count:    ", sap.sample_count)
  println(io, "  bw:              ", sap.bw)
  nothing
end

Base.show(io::IO, ::MIME"text/plain", sap::ScatterAlignPose2) = show(io, sap)
Base.show(io::IO, ::MIME"application/juno.inline", sap::ScatterAlignPose2) = show(io, sap)



## =========================================================================================
## Factor serialization below
## =========================================================================================

Base.@kwdef struct PackedScatterAlignPose2 <: AbstractPackedFactor
  cloud1::PackedHeatmapGridDensity # change to String
  cloud2::PackedHeatmapGridDensity # change to String
  gridscale::Float64 = 1.0
  sample_count::Int = 50
  bw::Float64 = 0.01
end

function convert(::Type{<:PackedScatterAlignPose2}, arp::ScatterAlignPose2)

  cloud1_ = packDistribution(arp.cloud1)
  cloud2_ = packDistribution(arp.cloud2)

  PackedScatterAlignPose2(;
    cloud1 = cloud1_,
    cloud2 = cloud2_,
    gridscale = arp.gridscale,
    sample_count = arp.sample_count,
    bw = arp.bw )
end

function convert(::Type{<:ScatterAlignPose2}, parp::PackedScatterAlignPose2)

  cloud1 = unpackDistribution(parp.cloud1)
  cloud2 = unpackDistribution(parp.cloud2)
  
  # and build the final object
  ScatterAlignPose2(
    cloud1,
    cloud2,
    parp.gridscale,
    parp.sample_count,
    parp.bw )
end

#