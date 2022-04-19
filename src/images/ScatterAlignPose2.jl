

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
  """ EXPERIMENTAL, flag whether to use 'stashing' for large point cloud, see [Stash & Cache](@ref section_stash_unstash) """
  useStashing::Bool = false
  """ DataEntry ID for hollow store of cloud 1 & 2 """
  dataEntry_cloud1::String = ""
  dataEntry_cloud2::String = ""
  """ Data store hint where likely to find the data entries and blobs for reconstructing cloud1 and cloud2"""
  dataStoreHint::String = ""
end


# replace inner constructor with transform on image
ScatterAlignPose2(im1::AbstractMatrix{T}, 
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
                ) where {T} = ScatterAlignPose2(;cloud1=HeatmapGridDensity(cvt(im1),domain,N=N), 
                                                cloud2=HeatmapGridDensity(cvt(im2),domain,N=N),
                                                gridscale=float(rescale),
                                                sample_count, 
                                                bw,
                                                useStashing,
                                                dataEntry_cloud1 = string(dataEntry_cloud1), 
                                                dataEntry_cloud2 = string(dataEntry_cloud2),
                                                dataStoreHint = string(dataStoreHint)  )
#

getManifold(::IIF.InstanceType{<:ScatterAlignPose2}) = getManifold(Pose2Pose2)

# runs once upon addFactor! and returns object later used as `cache`
function preambleCache(dfg::AbstractDFG, vars::AbstractVector{<:DFGVariable}, fnc::ScatterAlignPose2)
  #
  M = getManifold(Pose2)
  e0 = ProductRepr(SVector(0.0,0.0), SMatrix{2,2}(1.0, 0.0, 0.0, 1.0))

  # reconstitute cloud belief from dataEntry
  for (va,de,cl) in zip(vars,[fnc.dataEntry_cloud1,fnc.dataEntry_cloud2],[fnc.cloud1,fnc.cloud2])
    if fnc.useStashing 
      @assert 0 < length(de) "cannot reconstitute ScatterAlignPose2 without necessary data entry, only have $de"
      _, db = getData(dfg, getLabel(va), Symbol(de)) # fnc.dataStoreHint
      cld = convert(SamplableBelief, String(take!(IOBuffer(db))))
      # cld = unpackDistribution(strdstr)
      IIF._update!(cl, cld)
    end
  end

  smps1, = sample(fnc.cloud1, fnc.sample_count)
  smps2, = sample(fnc.cloud2, fnc.sample_count)

  bw = SA[fnc.bw;]
  
  score = Ref(0.0)

  (;M,e0,smps1,smps2,bw,score)
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
  res = Optim.optimize(cost, [5*randn(2); 0.1*randn()], Optim.BFGS() )
  
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
  sap = ScatterAlignPose2(;cloud1=sap_.cloud1, cloud2=sap_.cloud2, gridscale=sap_.gridscale, sample_count, bw=float(bw))
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
  _type::String = "Caesar.PackedScatterAlignPose2"
  cloud1::PackedHeatmapGridDensity
  cloud2::PackedHeatmapGridDensity
  gridscale::Float64 = 1.0
  sample_count::Int = 50
  bw::Float64 = 0.01
  """ EXPERIMENTAL, flag whether to use 'stashing' for large point cloud, see [Stash & Cache](@ref section_stash_unstash) """
  useStashing::Bool = false
  """ DataEntry ID for hollow store of cloud 1 & 2 """
  dataEntry_cloud1::String = ""
  dataEntry_cloud2::String = ""
  """ Data store hint where likely to find the data entries and blobs for reconstructing cloud1 and cloud2"""
  dataStoreHint::String = ""
end

function parchDistribution(mkd::ManifoldKernelDensity)
  pts = getPoints(mkd)
  bw = getBW(mkd)[:,1]
  manikde!(mkd.manifold, pts[1:1], mkd._u0; bw, partial=mkd._partial, infoPerCoord=mkd.infoPerCoord)
end

function parchDistribution(hgd::HeatmapGridDensity)
  @assert 2 <= size(hgd.data,1) "parchDistribution of HeatmapGridDensity can only be done when `.data` is larger than 2x1"
  
  data = Matrix{eltype(hgd.data)}(undef, 2,2)
  data[1,1] = hgd.data[1,1]
  # data[2,2] = hgd.data[2,2] # disable since data might be a single column in unusual cases
  data[2,1] = size(hgd.data,1)
  data[1,2] = size(hgd.data,2)

  domain = hgd.domain
  hint_callback = hgd.hint_callback
  bw_factor = hgd.bw_factor
  densityFnc = parchDistribution(hgd.densityFnc)
  
  HeatmapGridDensity(
    data,
    domain,
    hint_callback,
    bw_factor,
    densityFnc
  )
end


function convert(::Type{<:PackedScatterAlignPose2}, arp::ScatterAlignPose2)

  cld1 = arp.cloud1
  cld2 = arp.cloud2

  # reconstitute full type during the preambleCache step
  if arp.useStashing
    @assert length(arp.dataEntry_cloud1) !== 0 "packing of ScatterAlignPose2 asked to be `useStashing=true`` yet no `.dataEntry_cloud1` exists for later loading"
    cld1 = parchDistribution(arp.cloud1)
    @assert length(arp.dataEntry_cloud2) !== 0 "packing of ScatterAlignPose2 asked to be `useStashing=true`` yet no `.dataEntry_cloud2` exists for later loading"
    cld2 = parchDistribution(arp.cloud2)
  end

  cloud1 = packDistribution(cld1)
  cloud2 = packDistribution(cld2)

  PackedScatterAlignPose2(;
    cloud1,
    cloud2,
    gridscale = arp.gridscale,
    sample_count = arp.sample_count,
    bw = arp.bw,
    useStashing = arp.useStashing,
    dataEntry_cloud1 = arp.dataEntry_cloud1,
    dataEntry_cloud2 = arp.dataEntry_cloud2,
    dataStoreHint = arp.dataStoreHint )
end

function convert(::Type{<:ScatterAlignPose2}, parp::PackedScatterAlignPose2)
  #
  
  function _resizeCloudData!(cl::PackedHeatmapGridDensity)
    typ, col, row = typeof(cl.data[1][1]), Int(cl.data[1][2]), Int(cl.data[2][1])
    resize!(cl.data,row)
    for i in 1:row
      cl.data[i] = Vector{typ}(undef, col)
    end
    nothing
  end
  _resizeCloudData!(cl::ManifoldKernelDensity) = nothing
  
  # prep cloud1.data fields for larger data
  if parp.useStashing
    _resizeCloudData!(parp.cloud1)
    _resizeCloudData!(parp.cloud2)
  end
  
  cloud1 = unpackDistribution(parp.cloud1)
  cloud2 = unpackDistribution(parp.cloud2)
  
  # and build the final object
  ScatterAlignPose2(;
    cloud1,
    cloud2,
    gridscale=parp.gridscale,
    sample_count=parp.sample_count,
    bw=parp.bw,
    useStashing = parp.useStashing,
    dataEntry_cloud1 = parp.dataEntry_cloud1,
    dataEntry_cloud2 = parp.dataEntry_cloud2,
    dataStoreHint = parp.dataStoreHint 
  )
end

#