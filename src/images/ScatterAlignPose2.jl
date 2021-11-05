

import IncrementalInference: getSample
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
Base.@kwdef struct ScatterAlignPose2{ H1 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity},
                                      H2 <: Union{<:ManifoldKernelDensity, <:HeatmapGridDensity} } <: IIF.AbstractManifoldMinimize
  """ reference image for scan matching. """
  hgd1::H1
  """ test image to scan match against the reference image. """
  hgd2::H2
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
                ) where {T} = ScatterAlignPose2(HeatmapGridDensity(cvt(im1),domain,N=N), 
                                                HeatmapGridDensity(cvt(im2),domain,N=N),
                                                float(rescale),
                                                sample_count, bw  )
#

getManifold(::IIF.InstanceType{<:ScatterAlignPose2}) = getManifold(Pose2Pose2)

function getSample( cf::CalcFactor{<:ScatterAlignPose2} )
  #
  pts1, = sample(cf.factor.hgd1, cf.factor.sample_count)
  pts2, = sample(cf.factor.hgd2, cf.factor.sample_count)
  
  M = getManifold(Pose2)
  e0 = ProductRepr(SA[0 0.0], SMatrix{2,2}(1.0, 0, 0, 1))

  # precalc SE2 points
  R0 = e0.parts[2]

  pts2_ = map(pt->ProductRepr(pt, R0), pts2)

  return pts1, pts2_, M #, e0
end

function (cf::CalcFactor{<:ScatterAlignPose2})(Xtup, p, q)
  # 

  # get measured points 1 and 2, 
  # TODO move M to CalcFactor, follow IIF #1415
  pVi, qVj_, M = Xtup

  # get the current relative transform estimate
  pTq = Manifolds.compose(M, inv(M,p), q)
  
  # move other points with relative transform
  _pVj_ = map(pt->Manifolds.compose(M, pTq, pt).parts[1], qVj_ )


  return mmd(M.manifold[1], pVi, _pVj_, bw=SA[cf.factor.bw;])
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
    println(io, "  size(.data):     ", size(sap.hgd1.data))
    println(io, "    min/max:       ", round(minimum(sap.hgd1.data),digits=3), "/", round(maximum(sap.hgd1.data),digits=3))
  end
  if H2 <: HeatmapGridDensity
    println(io, "  size(.data):     ", size(sap.hgd2.data))
    println(io, "    min/max:       ", round(minimum(sap.hgd2.data),digits=3), "/", round(maximum(sap.hgd2.data),digits=3))
  end

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

  M = getManifold(Pose2Pose2)
  e0 = identity_element(M)
  R0 = e0.parts[2]

  # not efficient, but okay for here
  pTq(xyr=user_coords) = exp(M, e0, hat(M, e0, xyr))

  # sample points from the scatter field
  pts1, = sample(sap.hgd1, sample_count)
  pts2, = sample(sap.hgd2, sample_count)

  pts2_ = map(pt->ProductRepr(pt, R0), pts2)

  best_coords = zeros(3)
  if findBest
    # keep tfg separately so that optim can be more efficient
    tfg = initfg()
    ev_(xyr) = calcFactorResidualTemporary( sap, (Pose2,Pose2),
                                            (pts1,pts2_,M),
                                            (e0,pTq(xyr));
                                            tfg )[1]
    #
    score[] = ev_(user_coords)
    best = Optim.optimize(ev_, user_coords)
    best_coords .= round.(best.minimizer,digits=3)

    if showscore
      @info "overlayScatter score" score[] string(best_coords) best.minimum # user_offset
    end
  end

  # transform points1 to frame of 2 -- take p as coordinate expansion point
  pts2T_u = map(pt->Manifolds.compose(M, pTq(), pt).parts[1], pts2_)
  pts2T_b = map(pt->Manifolds.compose(M, pTq(best_coords), pt).parts[1], pts2_)

  @cast __pts1[i,j] := pts1[j][i]
  @cast __pts2[i,j] := pts2[j][i]
  @cast __pts2Tu[i,j] := pts2T_u[j][i]
  @cast __pts2Tb[i,j] := pts2T_b[j][i]

  (;pP1=__pts1, qP2=__pts2, pP2_u=__pts2Tu, pP2_b=__pts2Tb, best_coords, user_coords) #, user_offset)
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
  hgd1::String # PackedHeatmapGridDensity # change to String
  hgd2::String # PackedHeatmapGridDensity # change to String
  gridscale::Float64 = 1.0
  sample_count::Int = 50
  bw::Float64 = 0.01
end

function convert(::Type{<:PackedScatterAlignPose2}, arp::ScatterAlignPose2)
  hgd1 = convert(PackedHeatmapGridDensity,arp.hgd1)
  hgd2 = convert(PackedHeatmapGridDensity,arp.hgd2)
  hgd1_ = JSON2.write(hgd1)
  hgd2_ = JSON2.write(hgd2)

  PackedScatterAlignPose2(;
    hgd1 = hgd1_,
    hgd2 = hgd2_,
    gridscale = arp.gridscale,
    sample_count = arp.sample_count,
    bw = arp.bw )
end

function convert(::Type{<:ScatterAlignPose2}, parp::PackedScatterAlignPose2)
  # first understand the schema friendly belief type to unpack
  _hgd1 = JSON2.read(parp.hgd1)
  _hgd2 = JSON2.read(parp.hgd2)
  PackedT1 = DFG.getTypeFromSerializationModule(_hgd1[Symbol("_type")])
  PackedT2 = DFG.getTypeFromSerializationModule(_hgd2[Symbol("_type")])
  # re-unpack into the local PackedT (marshalling)
  # TODO check if there is perhaps optimization to marshal directly from _hgd instead - maybe `obj(;namedtuple...)`
  phgd1 = JSON2.read(parp.hgd1, PackedT1)
  phgd2 = JSON2.read(parp.hgd2, PackedT2)
  # convert from packed schema friendly to local performance type
  hgd1 = convert(SamplableBelief, phgd1)
  hgd2 = convert(SamplableBelief, phgd2)
  
  # and build the final object
  ScatterAlignPose2(
    hgd1,
    hgd2,
    parp.gridscale,
    parp.sample_count,
    parp.bw )
end

#