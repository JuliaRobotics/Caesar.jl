

import IncrementalInference: getSample, preambleCache, _update!
import Base: convert, show
# import DistributedFactorGraphs: getManifold
import ApproxManifoldProducts: sample, _update!

using UUIDs
using .Images

export ScatterAlignPose2, PackedScatterAlignPose2
export ScatterAlignPose3, PackedScatterAlignPose3

export overlayScatter, overlayScatterMutate

"""
    ScatterAlign{P,H1,H2} where  {H1 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity}, 
                                  H2 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity}}

Alignment factor between point cloud populations, using either
- a continuous density function cost: [`ApproxManifoldProducts.mmd`](@ref), or
- a conventional iterative closest point (ICP) algorithm (when `.sample_count < 0`).

This factor can support very large density clouds, with `sample_count` subsampling for individual alignments.

Keyword Options:
----------------
- `sample_count::Int = 100`, number of subsamples to use during each alignment in `getSample`.  
  - Values greater than 0 use MMD alignment, while values less than 0 use ICP alignment.
- `bw::Real`, the bandwidth to use for [`mmd`](@ref) distance
- `rescale::Real`
- `N::Int`
- `cvt::Function`, convert function for image when using `HeatmapGridDensity`.
- `useStashing::Bool = false`, to switch serialization strategy to using [Stashing](@ref section_stash_unstash).
- `dataEntry_cloud1::AbstractString = ""`, blob identifier used with stashing.
- `dataEntry_cloud2::AbstractString = ""`, blob identifier used with stashing.
- `dataStoreHint::AbstractString = ""`

Example
-------
```julia
arp2 = ScatterAlignPose2(img1, img2, 2) # e.g. 2 meters/pixel 
```

Notes
-----
- Supports two belief "clouds" as either
  - [`ManifoldKernelDensity`](@ref)s, or
  - [`HeatmapGridDensity`](@ref)s.
- Stanard `cvt` argument is lambda function to convert incoming images to user convention of image axes,
  - **Geography map default** `cvt` flips image rows so that Pose2 +xy-axes corresponds to img[-x,+y]
    - i.e. rows down is "North" and columns across from top left corner is "East".
- Use rescale to resize the incoming images for lower resolution (faster) correlations
- Both images passed to the construct must have the same type some matrix of type `T`.
- Experimental support for Stashing based serialization.

DevNotes:
---------
- TODO Upgrade to use other information during alignment process, e.g. point normals for Pose3.

See also: [`ScatterAlignPose2`](@ref), [`ScatterAlignPose3`](@ref), [`overlayScanMatcher`](@ref), [`Caesar._PCL.alignICP_Simple`](@ref).
"""
Base.@kwdef struct ScatterAlign{P,
                                H1 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity},
                                H2 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity} } <: IIF.AbstractManifoldMinimize
  """ reference image for scan matching. """
  cloud1::H1
  """ test image to scan match against the reference image. """
  cloud2::H2
  """ Common grid scale for both images -- i.e. units/pixel.  
  Constructor uses two arguments `gridlength`*`rescale=1`=`gridscale`.
  Arg 0 < `rescale` ≤ 1 is also used to rescale the images to lower resolution for speed. """
  gridscale::Float64 = 1.0
  """ how many heatmap sampled particles to use for mmd alignment """
  sample_count::Int  = 500
  """ bandwidth to use for mmd """
  bw::Float64        = 1.0
  """ EXPERIMENTAL, flag whether to use 'stashing' for large point cloud, see [Stash & Cache](@ref section_stash_unstash) """
  useStashing::Bool = false
  """ DataEntry ID for hollow store of cloud 1 & 2, TODO change to UUID instead """
  dataEntry_cloud1::String = ""
  dataEntry_cloud2::String = ""
  """ Data store hint where likely to find the data entries and blobs for reconstructing cloud1 and cloud2"""
  dataStoreHint::String = ""
end

"""
    ScatterAlignPose2(im1::Matrix, im2::Matrix, domain; options...)
    ScatterAlignPose2(; mkd1::ManifoldKernelDensity, mkd2::ManifoldKernelDensity, moreoptions...)

Specialization of [`ScatterAlign`](@ref) for [`Pose2`](@ref).

See also: [`ScatterAlignPose3`](@ref)
"""
struct ScatterAlignPose2 <: IIF.AbstractManifoldMinimize
  align::ScatterAlign{Pose2,<:Any,<:Any}
end

"""
    ScatterAlignPose3(; cloud1=mkd1::ManifoldKernelDensity, 
                        cloud2=mkd2::ManifoldKernelDensity, 
                        moreoptions...)

Specialization of [`ScatterAlign`](@ref) for [`Pose3`](@ref).

See also: [`ScatterAlignPose2`](@ref)
"""
struct ScatterAlignPose3 <: IIF.AbstractManifoldMinimize
  align::ScatterAlign{Pose3,<:Any,<:Any}
end

_getPoseType(::ScatterAlign{P}) where P = P

# replace inner constructor with transform on image
function ScatterAlignPose2(
    im1::AbstractMatrix{T}, 
    im2::AbstractMatrix{T},
    domain::Tuple{<:AbstractVector{<:Real},<:AbstractVector{<:Real}};
    sample_count::Integer=75,
    bw::Real=5e-5, # from a sensitivity analysis with marine radar data (50 or 100 samples)
    rescale::Real=1,
    N::Integer=1000,
    cvt = (im)->reverse(Images.imresize(im,trunc.(Int, rescale.*size(im))),dims=1),
    useStashing::Bool=false,
    dataEntry_cloud1="",
    dataEntry_cloud2="",
    dataStoreHint=""
  ) where {T}
  #
  cloud1 = HeatmapGridDensity(cvt(im1),domain,N=N)
  cloud2 = HeatmapGridDensity(cvt(im2),domain,N=N)
  sa = ScatterAlign{Pose2,typeof(cloud1),typeof(cloud2)}(;
                      cloud1,
                      cloud2,
                      gridscale=float(rescale),
                      sample_count, 
                      bw,
                      useStashing,
                      dataEntry_cloud1 = string(dataEntry_cloud1), 
                      dataEntry_cloud2 = string(dataEntry_cloud2),
                      dataStoreHint = string(dataStoreHint)  )
  #
  ScatterAlignPose2(sa)
end

function ScatterAlignPose2(;
    cloud1::ManifoldKernelDensity, 
    cloud2::ManifoldKernelDensity,
    sample_count::Integer=75,
    bw::Real=5e-5, # from a sensitivity analysis with marine radar data (50 or 100 samples)
    rescale::Real=1,
    useStashing::Bool=false,
    dataEntry_cloud1="",
    dataEntry_cloud2="",
    dataStoreHint=""
  )
  #
  
  sa = ScatterAlign{Pose2,typeof(cloud1),typeof(cloud2)}(;
                      cloud1,
                      cloud2,
                      gridscale=float(rescale),
                      sample_count, 
                      bw,
                      useStashing,
                      dataEntry_cloud1 = string(dataEntry_cloud1), 
                      dataEntry_cloud2 = string(dataEntry_cloud2),
                      dataStoreHint = string(dataStoreHint)  )
  #
  ScatterAlignPose2(sa)
end

function ScatterAlignPose3(;
    cloud1::ManifoldKernelDensity, 
    cloud2::ManifoldKernelDensity,
    sample_count::Integer=75,
    bw::Real=5e-5, # from a sensitivity analysis with marine radar data (50 or 100 samples)
    rescale::Real=1,
    useStashing::Bool=false,
    dataEntry_cloud1="",
    dataEntry_cloud2="",
    dataStoreHint=""
  )
  #
  
  sa = ScatterAlign{Pose3,typeof(cloud1),typeof(cloud2)}(;
                      cloud1,
                      cloud2,
                      gridscale=float(rescale),
                      sample_count, 
                      bw,
                      useStashing,
                      dataEntry_cloud1 = string(dataEntry_cloud1), 
                      dataEntry_cloud2 = string(dataEntry_cloud2),
                      dataStoreHint = string(dataStoreHint)  )
  #
  ScatterAlignPose3(sa)
end

function ScatterAlignPose3(
  ::Type{<:ManifoldKernelDensity};
  cloud1::_PCL.PointCloud,
  cloud2::_PCL.PointCloud,
  bw1 = [1;1;1.0],
  bw2 = bw1,
  kw...
)
  p1 = (s->s.data[1:3]).(cloud1.points) 
  p2 = (s->s.data[1:3]).(cloud2.points) 

  Mt = TranslationGroup(3)
  b1 = manikde!(Mt, p1; bw=bw1)
  b2 = manikde!(Mt, p2; bw=bw2)

  ScatterAlignPose3(;cloud1=b1, cloud2=b2, kw...)
end

getManifold(::IIF.InstanceType{<:ScatterAlignPose2}) = getManifold(Pose2Pose2)
getManifold(::IIF.InstanceType{<:ScatterAlignPose3}) = getManifold(Pose3Pose3)

# runs once upon addFactor! and returns object later used as `cache`
function preambleCache(
  dfg::AbstractDFG, 
  vars::AbstractVector{<:DFGVariable}, 
  fnc::Union{<:ScatterAlignPose2,<:ScatterAlignPose3}
)
  #
  M = getManifold(_getPoseType(fnc.align))
  e0 = getPointIdentity(M)

  # reconstitute cloud belief from dataEntry
  for (va,de,cl) in zip(vars,[fnc.align.dataEntry_cloud1,fnc.align.dataEntry_cloud2],[fnc.align.cloud1,fnc.align.cloud2])
    if fnc.align.useStashing 
      @assert 0 < length(de) "cannot reconstitute ScatterAlignPose2 without necessary data entry, only have $de"
      _, db = getData(dfg, getLabel(va), UUID(de)) # fnc.align.dataStoreHint
      # Assume PackedManifoldKernelDensity
      cld = convert(SamplableBelief, String(take!(IOBuffer(db))))
      # payload = JSON.parse(String(take!(IOBuffer(db))))
      # dstr = unmarshal PackedManifoldKernelDensity
      # cld = unpackDistribution(dstr)
      # update either a HGD or MKD
      _update!(cl, cld)
    end
  end

  smps1,smps2 = if 0 <= fnc.align.sample_count
    smps1, = sample(fnc.align.cloud1, fnc.align.sample_count)
    smps2, = sample(fnc.align.cloud2, fnc.align.sample_count)
    smps1, smps2
  else
    nothing, nothing
  end

  bw = SA[fnc.align.bw;]
  
  score = Ref(0.0)

  (;M,e0,smps1,smps2,bw,score)
end


function getSample( cf::CalcFactor{S} ) where {S <: Union{<:ScatterAlignPose2,<:ScatterAlignPose3}}
  #
  M = cf.cache.M
  e0 = cf.cache.e0
  ntr = length(Manifolds.submanifold_component(e0,1))
  nrt = Manifolds.manifold_dimension(M)-ntr
  # R0 = submanifold_component(e0,2)
  
  pVi = cf.cache.smps1
  qVj = cf.cache.smps2
  
  # TODO consolidate this transform with methods used by ICP
  pVj(qCp) = _transformPointCloud(M,qVj,qCp; backward=true)
  
  if 0 <= cf.factor.align.sample_count
    # Fresh samples
    for i in 1:cf.factor.align.sample_count
      pVi[i] .= sample(cf.factor.align.cloud1)[1][1]
      qVj[i] .= sample(cf.factor.align.cloud2)[1][1]
    end
    
    # qVi(qCp) = _transformPointCloud(M,pVi,qCp)
    # pVj(qCp) = _transformPointCloud(M,qVj,qCp; backward=true)
    # bw = SA[cf.factor.bw;]
    cost(xyr) = mmd(M.manifold[1], pVi, pVj(xyr), length(pVi), length(qVj), cf._allowThreads; cf.cache.bw)
    # return mmd as residual for minimization
    # FIXME upgrade to the on manifold optimization
    res = Optim.optimize(cost, [5*randn(ntr); 0.1*randn(nrt)], Optim.BFGS() )
    cf.cache.score[] = res.minimum
    # give measurement relative to e0 identity
    #  TODO relax to Riemannian where e0 is replaced by any point
    return hat(M, e0, res.minimizer)
  else #if cf.factor.align.sample_count < 0
    @assert cf.factor.align.cloud1 isa ManifoldKernelDensity "ICP alignments currently only implemented for beliefs as MKDs"
    ppt = getPoints(cf.factor.align.cloud1)
    qpt_ = getPoints(cf.factor.align.cloud2)

    # NOTE, duplicate code in ObjectAffordanceSubcloud
    # bump pc before alignment to get diversity in sample population
    dstb = 0.2*randn(getDimension(M)) # retract(M,e0,hat(M,e0,0.2*randn(getDimension(M))))
    qpt = _transformPointCloud(M, qpt_, dstb; backward=true)

    # FIXME, super inefficient repeat of data wrangling in hot loop
    @cast p_ptsM[i,d] := ppt[i][d]
    @cast phat_pts_mov[i,d] := qpt[i][d]

    # do actual alignment with ICP
    p_Hicp_phat, Hpts_mov, status = _PCL.alignICP_Simple(
      p_ptsM, 
      phat_pts_mov; 
      verbose=false,
      max_iterations = 25,
      correspondences = 500,
      neighbors = 50
    )
    # convert SE affine H to tangent vector X for use in residual function 
    p_P_q = ArrayPartition(p_Hicp_phat[1:end-1,end],p_Hicp_phat[1:end-1,1:end-1])
    # TODO confirm expansion around e0, since PosePose factors expand around `q`
    return log(M, e0, p_P_q)
  end
end


function (cf::CalcFactor{S})(X, p, q
  ) where {S <: Union{<:ScatterAlignPose2,<:ScatterAlignPose3}}
  # 
  
  M = cf.cache.M
  ϵ0 = cf.cache.e0
  
  # copied from Pose2Pose2
  q̂ = allocate(q)
  exp!(M, q̂, ϵ0, X)
  Manifolds.compose!(M, q̂, p, q̂)   
  Xc = vee(M, q, log!(M, q̂, q, q̂))
  return Xc

    # pXq, 
    # wPp, 
    # wPq
  # # get the current relative transform estimate
  # wPq_ = Manifolds.compose(M, wPp, exp(M, e0, pXq))
  # #TODO allocalte for vee! see Manifolds #412, fix for AD
  # Xc = zeros(3)
  # vee!(M, Xc, wPq, log(M, wPq, wPq_))
  # return Xc
end


"""
    $SIGNATURES

Overlay the two images from `AlignRadarPose2` with the first (red) fixed and transform the second image (blue) according to `tf`.

Notes:
- `tf` is a Manifolds.jl type `::ArrayPartition` (or newer `::ArrayPartition`) to represent a `SpecialEuclidean(2)` manifold point.
- `user_offset` translates both are the same.

See also: [`plotScatterAlign`](@ref)
"""
function overlayScatter(sap::ScatterAlignPose2, 
                        trans::AbstractVector{<:Real}=[0;0.0],
                        rot::Real=0.0;
                        user_coords = [trans; rot],
                        # score=Ref(0.0),
                        sample_count::Integer=sap.align.sample_count,
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
  pts2T_u = _transformPointCloud(M, PT2, user_coords; backward=true)
  pts2T_b = _transformPointCloud(M, PT2, best; backward=true)

  @cast __pts1[i,j] := PT1[j][i]
  @cast __pts2[i,j] := PT2[j][i]
  @cast __pts2Tu[i,j] := pts2T_u[j][i]
  @cast __pts2Tb[i,j] := pts2T_b[j][i]
  
  # return NamedTuple obj with all the necessary data for plotting, see plotScatterAlign
  (;pP1=__pts1, qP2=__pts2, pP2_u=__pts2Tu, pP2_b=__pts2Tb, best_coords, user_coords, u_score=round(0,sigdigits=3), b_score=round(b_score[],sigdigits=3)) #, user_offset)
end


function overlayScatterMutate(sap_::ScatterAlignPose2;
                              sample_count::Integer = sap_.align.sample_count,
                              bw::Real = sap_.align.bw,
                              kwargs... )
  #
  sap = ScatterAlign{Pose2,typeof(sap_.align.cloud1),typeof(sap_.align.cloud2)}(;cloud1=sap_.align.cloud1, cloud2=sap_.align.cloud2, gridscale=sap_.align.gridscale, sample_count, bw=float(bw))
  sa = ScatterAlignPose2(sap)
  Caesar.overlayScatter(sa; kwargs...);
end


function Base.show(io::IO, sap::ScatterAlign{P,H1,H2}) where {P,H1,H2}
  printstyled(io, "ScatterAlign{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    P  = ", color=:magenta)
  println(io, P)
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

Base.show(io::IO, ::MIME"text/plain", sap::ScatterAlign) = show(io, sap)
Base.show(io::IO, ::MIME"application/juno.inline", sap::ScatterAlign) = show(io, sap)

Base.show(io::IO, ::MIME"text/plain", sap::ScatterAlignPose2) = show(io, sap.align)
Base.show(io::IO, ::MIME"application/juno.inline", sap::ScatterAlignPose2) = show(io, sap.align)


## =========================================================================================
## Factor serialization below
## =========================================================================================


const _PARCHABLE_PACKED_CLOUD = Union{<:PackedManifoldKernelDensity, <:PackedHeatmapGridDensity}

Base.@kwdef struct PackedScatterAlignPose2 <: AbstractPackedFactor
  _type::String = "Caesar.PackedScatterAlignPose2"
  cloud1::_PARCHABLE_PACKED_CLOUD
  cloud2::_PARCHABLE_PACKED_CLOUD
  gridscale::Float64 = 1.0
  sample_count::Int = 50
  bw::Float64 = 0.01
  """ EXPERIMENTAL, flag whether to use 'stashing' for large point cloud, see [Stash & Cache](@ref section_stash_unstash) """
  useStashing::Bool = false
  """ DataEntry ID for stash store of cloud 1 & 2 """
  dataEntry_cloud1::String = ""
  dataEntry_cloud2::String = ""
  """ Data store hint where likely to find the data entries and blobs for reconstructing cloud1 and cloud2"""
  dataStoreHint::String = ""
end

Base.@kwdef struct PackedScatterAlignPose3 <: AbstractPackedFactor
  _type::String = "Caesar.PackedScatterAlignPose3"
  cloud1::_PARCHABLE_PACKED_CLOUD
  cloud2::_PARCHABLE_PACKED_CLOUD
  gridscale::Float64 = 1.0
  sample_count::Int = 50
  bw::Float64 = 0.01
  """ EXPERIMENTAL, flag whether to use 'stashing' for large point cloud, see [Stash & Cache](@ref section_stash_unstash) """
  useStashing::Bool = false
  """ DataEntry ID for stash store of cloud 1 & 2 """
  dataEntry_cloud1::String = ""
  dataEntry_cloud2::String = ""
  """ Data store hint where likely to find the data entries and blobs for reconstructing cloud1 and cloud2"""
  dataStoreHint::String = ""
end

function convert(
  ::Type{T}, 
  arp::Union{<:ScatterAlignPose2,<:ScatterAlignPose3}
) where {T <: Union{<:PackedScatterAlignPose2,<:PackedScatterAlignPose3}}
  #
  cld1 = arp.align.cloud1
  cld2 = arp.align.cloud2

  # reconstitute full type during the preambleCache step
  if arp.align.useStashing
    @assert length(arp.align.dataEntry_cloud1) !== 0 "packing of ScatterAlignPose asked to be `useStashing=true`` yet no `.dataEntry_cloud1` exists for later loading"
    cld1 = IIF.parchDistribution(arp.align.cloud1)
    @assert length(arp.align.dataEntry_cloud2) !== 0 "packing of ScatterAlignPose asked to be `useStashing=true`` yet no `.dataEntry_cloud2` exists for later loading"
    cld2 = IIF.parchDistribution(arp.align.cloud2)
  end

  cloud1 = packDistribution(cld1)
  cloud2 = packDistribution(cld2)

  T(;
    cloud1,
    cloud2,
    gridscale = arp.align.gridscale,
    sample_count = arp.align.sample_count,
    bw = arp.align.bw,
    useStashing = arp.align.useStashing,
    dataEntry_cloud1 = arp.align.dataEntry_cloud1,
    dataEntry_cloud2 = arp.align.dataEntry_cloud2,
    dataStoreHint = arp.align.dataStoreHint 
  )
end

function convert(
  ::Type{T}, 
  parp::Union{<:PackedScatterAlignPose2,<:PackedScatterAlignPose3}
) where {T <: Union{<:ScatterAlignPose2,<:ScatterAlignPose3}}
  #
  _selectPoseT(::Type{<:ScatterAlignPose2}) = Pose2
  _selectPoseT(::Type{<:ScatterAlignPose3}) = Pose3

  function _resizeCloudData!(cl::PackedHeatmapGridDensity)
    row=Int(cl.data[2][1])
    typ, col = typeof(cl.data[1][1]), Int(cl.data[1][2])
    resize!(cl.data,row)
    for i in 1:row
      cl.data[i] = Vector{typ}(undef, col)
    end
    nothing
  end
  _resizeCloudData!(cl::PackedManifoldKernelDensity) = nothing

  # prep cloud1.data fields for larger data
  if parp.useStashing
    _resizeCloudData!(parp.cloud1)
    _resizeCloudData!(parp.cloud2)
  end
  
  cloud1 = unpackDistribution(parp.cloud1)
  cloud2 = unpackDistribution(parp.cloud2)
  
  # and build the final object
  poseT = _selectPoseT(T)
  T(ScatterAlign{poseT, typeof(cloud1), typeof(cloud2)}(;
    cloud1,
    cloud2,
    gridscale=parp.gridscale,
    sample_count=parp.sample_count,
    bw=parp.bw,
    useStashing = parp.useStashing,
    dataEntry_cloud1 = parp.dataEntry_cloud1,
    dataEntry_cloud2 = parp.dataEntry_cloud2,
    dataStoreHint = parp.dataStoreHint 
  ))
end

#