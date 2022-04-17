

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
  
  (;M,e0,smps1,smps2)
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

function getSample( cf::CalcFactor{<:ScatterAlignPose2} )
  #
  M = cf.cache.M   # getManifold(Pose2)
  e0 = cf.cache.e0 # ProductRepr(SVector(0.0,0.0), SMatrix{2,2}(1.0, 0.0, 0.0, 1.0))
  
  pVi,  = sample(cf.factor.cloud1, cf.factor.sample_count)
  pts2, = sample(cf.factor.cloud2, cf.factor.sample_count)

  # precalc SE2 points
  R0 = e0.parts[2]
  # source memory
  qVj_ = map(pt->ProductRepr(SVector(pt...), R0), pts2)
  # destination memory
  _pVj_! = map(pt->ProductRepr(MVector(pt...), MMatrix{2,2}(0.0,0.0,0.0,0.0)), pts2)
  _pVj! = map(x->x.parts[1], _pVj_!)

  function pVj(xyr)
    pTq! = _FastRetract()
    for i in eachindex(qVj_)
      Manifolds.compose!(M, _pVj_![i], pTq!(xyr), qVj_[i])
      # _pVj![i][:] = Manifolds.compose(M, pTq!(xyr), qVj_[i]).parts[1] 
    end

    _pVj!
  end

  # # get the current relative transform estimate
  # pTq! = _FastRetract() # not really any faster yet
  # # not efficient, but okay for here
  # # pTq(xyr) = exp(M, e0, hat(M, e0, xyr))
  # # move other points with relative transform
  # pVj(xyr) = map(pt->Manifolds.compose(M, pTq!(xyr), pt).parts[1], qVj_)
  
  bw = SA[cf.factor.bw;]
  cost(xyr) = mmd(M.manifold[1], pVi, pVj(xyr), length(pVi), length(qVj_), cf._allowThreads; bw)

  # return mmd as residual for minimization
  res = Optim.optimize(cost, [10*randn(2); 0.1*randn()] )
  
  M, e0, hat(M, e0, res.minimizer)
end


function (cf::CalcFactor{<:ScatterAlignPose2})(Xtup, wPp, wPq)
  # 

  # TODO move M to CalcFactor, follow IIF #1415
  M, e0, pXq = Xtup

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
                        score=Ref(0.0),
                        sample_count::Integer=sap.sample_count,
                        showscore::Bool=true,
                        findBest::Bool=true  )
  #

  # # sample points from the scatter field
  tfg = initfg()
  addVariable!(tfg, :x0, Pose2)
  addVariable!(tfg, :x1, Pose2)
  addFactor!(tfg, [:x0;:x1], sap; graphinit=false)

  meas = sampleFactor(tfg, :x0x1f1)[1]
  M, pts1, pts2_ = meas
  e0 = identity_element(M)
  PT1 = []
  PT2_ = []
  PT2 = []
  for k in 1:sample_count
    meas = sampleFactor(tfg, :x0x1f1)[1]
    _, pts1_, pts2_ = meas
    push!(PT1, pts1_.parts[1])
    push!(PT2_, pts2_)
    push!(PT2, pts2_.parts[1])
  end
  # pts2 = map(pt->pt.parts[1], pts2_)

  # not efficient, but okay for here
  pTq(xyr=user_coords) = exp(M, e0, hat(M, e0, xyr))


  best_coords = zeros(3)
  if findBest
    # keep tfg separately so that optim can be more efficient
    # tfg = initfg()
    ev_(xyr) = calcFactorResidualTemporary( sap, (Pose2,Pose2),
                                            meas,
                                            (e0,pTq(xyr));
                                            tfg )[1]
    #
    score[] = ev_(user_coords)
    best = Optim.optimize(ev_, user_coords, BFGS())
    best_coords .= round.(best.minimizer,digits=3)

    if showscore
      @info "overlayScatter score" score[] string(best_coords) best.minimum # user_offset
    end
  end

  # transform points1 to frame of 2 -- take p as coordinate expansion point
  pts2T_u = map(pt->Manifolds.compose(M, inv(M, pTq()), pt).parts[1], PT2_)
  pts2T_b = map(pt->Manifolds.compose(M, inv(M, pTq(best_coords)), pt).parts[1], PT2_)

  @cast __pts1[i,j] := PT1[j][i]
  @cast __pts2[i,j] := PT2[j][i]
  @cast __pts2Tu[i,j] := pts2T_u[j][i]
  @cast __pts2Tb[i,j] := pts2T_b[j][i]

  (;pP1=__pts1, qP2=__pts2, pP2_u=__pts2Tu, pP2_b=__pts2Tb, best_coords, user_coords, score=round(score[],digits=5)) #, user_offset)
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

  function _toDensityJson(dens::ManifoldKernelDensity)
    # convert(PackedSamplableBelief,dens)
    packDistribution(dens)
    # JSON2.write(pd)
  end
  function _toDensityJson(dens::HeatmapGridDensity)
    packDistribution(dens)
    # cloud1 = convert(PackedHeatmapGridDensity,dens)
    # JSON2.write(cloud1)    
  end

  cloud1_ = _toDensityJson(arp.cloud1)
  cloud2_ = _toDensityJson(arp.cloud2)

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

  # # first understand the schema friendly belief type to unpack
  # _cloud1 = JSON2.read(parp.cloud1)
  # _cloud2 = JSON2.read(parp.cloud2)
  # PackedT1 = DFG.getTypeFromSerializationModule(_cloud1[Symbol("_type")])
  # PackedT2 = DFG.getTypeFromSerializationModule(_cloud2[Symbol("_type")])
  # # outT
  # @info "HERE" PackedT1
  # outT1 = convert(SamplableBelief, PackedT1)
  # outT2 = convert(SamplableBelief, PackedT2)
  # # convert from packed schema friendly to local performance type
  # cloud1 = convert(outT1, parp.cloud1)
  # cloud2 = convert(outT2, parp.cloud2)
  
  # and build the final object
  ScatterAlignPose2(
    cloud1,
    cloud2,
    parp.gridscale,
    parp.sample_count,
    parp.bw )
end

#