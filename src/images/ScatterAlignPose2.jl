

import IncrementalInference: getSample
import Base: convert, show
# import DistributedFactorGraphs: getManifold

using .Images

export ScatterAlignPose2, PackedScatterAlignPose2

export overlayScatter, overlayScatterMutate

"""
$TYPEDEF

This is but one incarnation for how radar alignment factor could work, treat it as a starting point.

Notes
- Stanard `cvt` argument is lambda function to convert incoming images to user convention of image axes,
  - default `cvt` flips image rows so that Pose2 xy-axes corresponds to img[x,y] -- i.e. rows down and across from top left corner.
- Use rescale to resize the incoming images for lower resolution (faster) correlations
- Both images passed to the construct must have the same type some matrix of type `T`.

Example
-------
```julia
arp2 = ScatterAlignPose2(img1, img2, 2) # e.g. 2 meters/pixel 
```

See also: [`overlayScanMatcher`](@ref)
"""
struct ScatterAlignPose2{H1<:HeatmapGridDensity,H2<:HeatmapGridDensity} <: IIF.AbstractManifoldMinimize
  """ reference image for scan matching. """
  hgd1::H1
  """ test image to scan match against the reference image. """
  hgd2::H2
  """ Common grid scale for both images -- i.e. units/pixel.  
  Constructor uses two arguments `gridlength`*`rescale=1`=`gridscale`.
  Arg 0 < `rescale` ≤ 1 is also used to rescale the images to lower resolution for speed. """
  gridscale::Float64
  """ how many heatmap sampled particles to use for mmd ailgnment """
  sample_count::Int
  """ bandwidth to use for mmd """
  bw::Float64
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
                  ) where {T} = ScatterAlignPose2(HeatmapGridDensity(cvt(im1),domain,N=N), 
                                                  HeatmapGridDensity(cvt(im2),domain,N=N),
                                                  float(rescale),
                                                  sample_count, bw  )
#

getManifold(::IIF.InstanceType{<:ScatterAlignPose2}) = getManifold(Pose2Pose2)

function getSample( cf::CalcFactor{<:ScatterAlignPose2} )
  
  pts1, = sample(cf.factor.hgd1.densityFnc, cf.factor.sample_count)
  pts2, = sample(cf.factor.hgd1.densityFnc, cf.factor.sample_count)
  
  M = getManifold(Pose2)
  e0 = ProductRepr(SA[0 0.0], SMatrix{2,2}(1.0, 0, 0, 1)) # identity_element(M)

  # precalc SE2 points
  R0 = e0.parts[2]

  ## FIXME REMOVE DEBUG + [20.0;-10]
  pts2_ = map(pt->ProductRepr(pt, R0), pts2)

  return pts1, pts2_, M #, e0
end

function (cf::CalcFactor{<:ScatterAlignPose2})(Xtup, p, q)
  # 

  # will move to CalcFactor, follow IIF #1415
  M = Xtup[3]

  pts1, pts2_ = Xtup[1], Xtup[2]

  # get the current relative transform estimate
  pTq = Manifolds.compose(M, inv(M,p), q)
  
  # move other points with relative transform
  pts2__ = map(i->Manifolds.compose(M, pTq, pts2_[i]).parts[1], 1:length(pts1))
  
  # cf._residual[1] = 
  return Float64[mmd(M.manifold[1], pts1, pts2__, bw=SA[cf.factor.bw;]);]
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
  println(io, "  size(.data):     ", size(sap.hgd1.data))
  println(io, "    min/max:       ", round(minimum(sap.hgd1.data),digits=3), "/", round(maximum(sap.hgd1.data),digits=3))

  println(io, "  size(.data):     ", size(sap.hgd2.data))
  println(io, "    min/max:       ", round(minimum(sap.hgd2.data),digits=3), "/", round(maximum(sap.hgd2.data),digits=3))

  println(io, "  gridscale:       ", sap.gridscale)
  println(io, "  sample_count:    ", sap.sample_count)
  println(io, "  bw:              ", sap.bw)
  nothing
end

Base.show(io::IO, ::MIME"text/plain", sap::ScatterAlignPose2) = show(io, sap)
Base.show(io::IO, ::MIME"application/juno.inline", sap::ScatterAlignPose2) = show(io, sap)





"""
    $SIGNATURES

Overlay the two images from `AlignRadarPose2` with the first (red) fixed and transform the second image (blue) according to `tf`.

Notes:
- `tf` is a Manifolds.jl type `::ProductRepr` (or newer `::ArrayPartition`) to represent a `SpecialEuclidean(2)` manifold point.
"""
function overlayScatter(sap::ScatterAlignPose2, 
                        trans::AbstractVector{<:Real}=SA[0;0.0],
                        rot::Real=0.0;
                        user_coords = [trans; rot],
                        offsetTrans::AbstractVector{<:Real}=SA[0;0.0],
                        user_offset = [offsetTrans;0.0],
                        score=Ref(0.0),
                        sample_count::Integer=sap.sample_count,
                        showscore::Bool=true,
                        findBest::Bool=true  )
  #

  pts1_, = sample(sap.hgd1.densityFnc, sample_count)
  pts2_, = sample(sap.hgd2.densityFnc, sample_count)
  
  # artificial offset for testing
  # TODO must still add the rotation user_offset component (not critical)
  for pt in pts2_ 
    pt .+= user_offset[1:2]
  end

  M = getManifold(Pose2Pose2)
  e0 = identity_element(M)
  # not efficient, but okay for here
  pTq(xyr=user_coords) = exp(M, e0, hat(M, e0, xyr))

  R0 = e0.parts[2]
  _pts2_ = map(pt->ProductRepr(pt, R0), pts2_)

  # take p as reference identity
  pts2T_ = map(pt->Manifolds.compose(M, pTq(), pt).parts[1], _pts2_)
  best_coords = zeros(3)

  if findBest
    # keep tfg separately so that optim can be more efficient
    tfg = initfg()
    ev_(xyr) = calcFactorResidualTemporary( sap, (Pose2,Pose2),
                                            (pts1_,_pts2_,M),
                                            (e0,pTq(xyr));
                                            tfg )[1]
    #
    score[] = ev_(user_coords)
    best = Optim.optimize(ev_, user_coords)
    best_coords .= round.(best.minimizer,digits=3)

    if showscore
      @info "overlayScatter score" score[] string(best_coords) best.minimum user_offset
    end
  end

  pts2T_best = map(pt->Manifolds.compose(M, pTq(best_coords), pt).parts[1], _pts2_)

  @cast pts1[i,j] := pts1_[j][i]
  @cast pts2[i,j] := pts2_[j][i]
  @cast pts2T[i,j] := pts2T_[j][i]
  @cast pts2Tb[i,j] := pts2T_best[j][i]

  (;pPp=pts1, qPq=pts2, pPq=pts2T, pPq_best=pts2Tb, best_coords, user_coords, user_offset)
end


function overlayScatterMutate(sap_::ScatterAlignPose2;
                              sample_count::Integer = sap_.sample_count,
                              bw::Real = sap_.bw,
                              kwargs... )
  #
  sap = ScatterAlignPose2(sap_.hgd1, sap_.hgd2, sap_.gridscale, sample_count, float(bw))
  Caesar.overlayScatter(sap; kwargs...);
end


## =========================================================================================
## Factor serialization below
## =========================================================================================

Base.@kwdef struct PackedScatterAlignPose2 <: PackedInferenceType
  hgd1::PackedHeatmapGridDensity
  hgd2::PackedHeatmapGridDensity
  gridscale::Float64 = 1.0
  sample_count::Int = 50
  bw::Float64 = 0.01
end

function convert(::Type{<:PackedScatterAlignPose2}, arp::ScatterAlignPose2)
  PackedScatterAlignPose2(;
    hgd1 = convert(PackedHeatmapGridDensity,arp.hgd1),
    hgd2 = convert(PackedHeatmapGridDensity,arp.hgd2),
    gridscale = arp.gridscale,
    sample_count = arp.sample_count,
    bw = arp.bw )
end

function convert(::Type{<:ScatterAlignPose2}, parp::PackedScatterAlignPose2)
  ScatterAlignPose2(
    covert(HeatmapGridDensity,parp.hgd1),
    covert(HeatmapGridDensity,parp.hgd2),
    parp.gridscale,
    parp.sample_count,
    parp.bw )
end

#