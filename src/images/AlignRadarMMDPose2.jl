

import IncrementalInference: getSample
import Base: convert, show
# import DistributedFactorGraphs: getManifold

using .Images

export AlignRadarMMDPose2, PackedAlignRadarMMDPose2

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
arp2 = AlignRadarMMDPose2(img1, img2, 2) # e.g. 2 meters/pixel 
```

See also: [`overlayScanMatcher`](@ref)
"""
struct AlignRadarMMDPose2{H1<:HeatmapGridDensity,H2<:HeatmapGridDensity} <: IIF.AbstractManifoldMinimize
  """ reference image for scan matching. """
  hgd1::H1
  """ test image to scan match against the reference image. """
  hgd2::H2
  """ Common grid scale for both images -- i.e. units/pixel.  
  Constructor uses two arguments `gridlength`*`rescale=1`=`gridscale`.
  Arg 0 < `rescale` ≤ 1 is also used to rescale the images to lower resolution for speed. """
  gridscale::Float64

end

  # replace inner constructor with transform on image
  AlignRadarMMDPose2( im1::AbstractMatrix{T}, 
                      im2::AbstractMatrix{T},
                      domain::Tuple{<:AbstractVector{<:Real},<:AbstractVector{<:Real}};
                      rescale::Real=1,
                      N::Integer=1000,
                      cvt = (im)->reverse(Images.imresize(im,trunc.(Int, rescale.*size(im))),dims=1)
                    ) where {T} = AlignRadarMMDPose2( HeatmapGridDensity(cvt(im1),domain,N=N), 
                                                      HeatmapGridDensity(cvt(im2),domain,N=N),
                                                      float(rescale) )
#

getManifold(::IIF.InstanceType{<:AlignRadarMMDPose2}) = getManifold(Pose2Pose2)

function getSample( cf::CalcFactor{<:AlignRadarMMDPose2} )
  
  pts1, = sample(cf.factor.hgd1.densityFnc, 100)
  pts2, = sample(cf.factor.hgd1.densityFnc, 100)
  
  M = getManifold(Pose2)
  e0 = ProductRepr(SA[0 0.0], SMatrix{2,2}(1.0, 0, 0, 1)) # identity_element(M)

  # precalc SE2 points
  R0 = e0.parts[2]
  pts2_ = map(pt->ProductRepr(pt, R0), pts2)

  # @info "HERE" typeof(pts2) length(pts2) typeof(pts2_) length(pts2_)

  return pts1, pts2_ #, M, e0
end

function (cf::CalcFactor{<:AlignRadarMMDPose2})(Xtup, p, q)
  # 
  M = getManifold(Pose2)
  # e0 = ProductRepr(SA[0 0.0], SMatrix{2,2}(1.0, 0, 0, 1)) # identity_element(M)

  pts1, pts2_ = Xtup[1], Xtup[2]
  # M = Xtup[3]

  # get the current relative transform estimate
  pTq = Manifolds.compose(M, inv(M,p), q)
  
  # move other points with relative transform
  pts2__ = map(i->Manifolds.compose(M, pTq, pts2_[i]).parts[1], 1:length(pts1))
  
  return Float64[mmd(M.manifold[1], pts1, pts2__);]
  
  # arp = cf.factor
  # # for groups
  # tf = Manifolds.compose(M, inv(M, p), q) 
  # Mr = M.manifold[2]
  # r0 = identity_element(Mr)
  # r = vee(Mr, r0, log(Mr, r0, tf.parts[2]))[1]
  # return # evaluateTransform(arp.im1, arp.im2, tf.parts[1], r, arp.gridscale)
end

# function Base.show(io::IO, arp::AlignRadarMMDPose2{T}) where {T}
#   printstyled(io, "AlignRadarMMDPose2{", bold=true, color=:blue)
#   println(io)
#   printstyled(io, "    T = ", color=:magenta)
#   println(io, T)
#   printstyled(io, " }", color=:blue, bold=true)
#   println(io)
#   println(io, "  size(.im1):      ", size(arp.im1))
#   println(io, "    min/max:       ", round(minimum(arp.im1),digits=3), "/", round(maximum(arp.im1),digits=3))
#   println(io, "  size(.im2):      ", size(arp.im2))
#   println(io, "    min/max:       ", round(minimum(arp.im2),digits=3), "/", round(maximum(arp.im2),digits=3))
#   println(io, "  gridscale:       ", arp.gridscale)
#   nothing
# end

# Base.show(io::IO, ::MIME"text/plain", arp::AlignRadarMMDPose2) = show(io, arp)
# Base.show(io::IO, ::MIME"application/juno.inline", arp::AlignRadarMMDPose2) = show(io, arp)


"""
    $SIGNATURES

Overlay the two images from `AlignRadarPose2` with the first (red) fixed and transform the second image (blue) according to `tf`.

Notes:
- `tf` is a Manifolds.jl type `::ProductRepr` (or newer `::ArrayPartition`) to represent a `SpecialEuclidean(2)` manifold point.
"""
function overlayScanMatcher(sm::AlignRadarMMDPose2, 
                            trans::AbstractVector{<:Real}=Float64[0;0],
                            rot::Real=0.0;
                            score=Ref(0.0),
                            showscore::Bool=true  )
  #
  # im2_ = transformImage_SE2(sm.im2, trans, rot, sm.gridscale)
  
  # # get matching padded views
  # im1_, im2__ = paddedviews(0.0, sm.im1, im2_)

  # im1__ = RGBA.(im1_, 0, 0, 0.5)
  # im2___ = RGBA.(0, 0, im2__, 0.5)    

  # if showscore
  #   score[] = evaluateTransform(sm.im1, sm.im2, trans, rot, sm.gridscale)
  #   @info "overlayScanMatcher score" score[]
  # end

  # # im2__ = RGBA.(im2_, 0, 0, 0.5)
  # im1__ .+ im2___
end


#

function overlayScanMatcher(sm::AlignRadarMMDPose2, 
                            tf = Manifolds.identity_element(SpecialEuclidean(2));
                            kw... )
  #
  # M = SpecialOrthogonal(2)
  # e0 = identity_element(M)

  # rot = vee( M, e0, log(M, e0, tf.parts[2]) )[1]

  # overlayScanMatcher(sm, tf.parts[1], rot; kw...)
end

## =========================================================================================
## Factor serialization below
## =========================================================================================

struct PackedAlignRadarMMDPose2 <: PackedInferenceType
  hgd1::PackedHeatmapGridDensity
  hgd2::PackedHeatmapGridDensity
  gridscale::Float64
end

function convert(::Type{<:PackedAlignRadarMMDPose2}, arp::AlignRadarMMDPose2)
  PackedAlignRadarMMDPose2(
    convert(PackedHeatmapGridDensity,arp.hgd1),
    convert(PackedHeatmapGridDensity,arp.hgd2),
    arp.gridscale )
end

function convert(::Type{<:AlignRadarMMDPose2}, parp::PackedAlignRadarMMDPose2)
  AlignRadarMMDPose2(
    covert(HeatmapGridDensity,parp.hgd1),
    covert(HeatmapGridDensity,parp.hgd2),
    parp.gridscale )
end

#