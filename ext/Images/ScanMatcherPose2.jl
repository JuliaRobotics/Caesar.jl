



ScanMatcherPose2(im1::AbstractMatrix{T}, im2::AbstractMatrix{T}, w...) where T = ScanMatcherPose2{T}(im1,im2, w...)

getManifold(::IIF.InstanceType{<:ScanMatcherPose2}) = getManifold(Pose2Pose2)

getSample( cf::CalcFactor{<:ScanMatcherPose2} ) = nothing

function (cf::CalcFactor{<:ScanMatcherPose2})(X, p, q)
  M = getManifold(Pose2)
  arp = cf.factor
  # for groups
  tf = Manifolds.compose(M, inv(M, p), q) 
  Mr = M.manifold[2]
  r0 = identity_element(Mr)
  r = vee(Mr, r0, log(Mr, r0, tf.parts[2]))[1]
  return evaluateTransform(arp.im1, arp.im2, tf.parts[1], r, arp.gridscale)
end

function Base.show(io::IO, arp::ScanMatcherPose2{T}) where {T}
  printstyled(io, "ScanMatcherPose2{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    T = ", color=:magenta)
  println(io, T)
  printstyled(io, " }", color=:blue, bold=true)
  println(io)
  println(io, "  size(.im1):      ", size(arp.im1))
  println(io, "    min/max:       ", round(minimum(arp.im1),digits=3), "/", round(maximum(arp.im1),digits=3))
  println(io, "  size(.im2):      ", size(arp.im2))
  println(io, "    min/max:       ", round(minimum(arp.im2),digits=3), "/", round(maximum(arp.im2),digits=3))
  println(io, "  gridscale:       ", arp.gridscale)
  nothing
end

Base.show(io::IO, ::MIME"text/plain", arp::ScanMatcherPose2) = show(io, arp)
Base.show(io::IO, ::MIME"application/juno.inline", arp::ScanMatcherPose2) = show(io, arp)


"""
    $SIGNATURES

Overlay the two images from `AlignRadarPose2` with the first (red) fixed and transform the second image (blue) according to `tf`.

Notes:
- `tf` is a Manifolds.jl type `::ProductRepr` (or newer `::ArrayPartition`) to represent a `SpecialEuclidean(2)` manifold point.
"""
function overlayScanMatcher(sm::ScanMatcherPose2, 
                            trans::Vector{Float64}=Float64[0;0],
                            rot::Real=0.0;
                            score=Ref(0.0),
                            showscore::Bool=true  )
  #
  im2_ = transformImage_SE2(sm.im2, trans, rot, sm.gridscale)
  
  # get matching padded views
  im1_, im2__ = paddedviews(0.0, sm.im1, im2_)

  im1__ = RGBA.(im1_, 0, 0, 0.5)
  im2___ = RGBA.(0, 0, im2__, 0.5)    

  if showscore
    score[] = evaluateTransform(sm.im1, sm.im2, trans, rot, sm.gridscale)
    @info "overlayScanMatcher score" score[]
  end

  # im2__ = RGBA.(im2_, 0, 0, 0.5)
  im1__ .+ im2___
end


#

function overlayScanMatcher(sm::ScanMatcherPose2, 
                            tf::ArrayPartition = Manifolds.identity_element(SpecialEuclidean(2));
                            kw... )
  #
  M = SpecialOrthogonal(2)
  e0 = identity_element(M)

  rot = vee( M, e0, log(M, e0, tf.parts[2]) )[1]

  overlayScanMatcher(sm, tf.parts[1], rot; kw...)
end

## =========================================================================================
## Factor serialization below
## =========================================================================================


function convert(::Type{<:PackedScanMatcherPose2}, arp2::ScanMatcherPose2)
  TensorCast.@cast pim1[row][col] := arp2.im1[row,col]
  TensorCast.@cast pim1[row] := collect(pim1[row])
  TensorCast.@cast pim2[row][col] := arp2.im2[row,col]
  TensorCast.@cast pim2[row] := collect(pim2[row])
  PackedScanMatcherPose2(
    pim1,
    pim2,
    arp2.gridscale )
end

function convert(::Type{<:ScanMatcherPose2}, parp2::PackedScanMatcherPose2)
  TensorCast.@cast im1[row,col] := parp2.im1[row][col]
  TensorCast.@cast im2[row,col] := parp2.im2[row][col]
  ScanMatcherPose2(
    collect(im1),
    collect(im2),
    parp2.gridscale )
end

#