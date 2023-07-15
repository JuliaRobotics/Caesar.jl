

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
arp2 = ScanMatcherPose2(img1, img2, 2) # e.g. 2 meters/pixel 
```

See also: [`overlayScanMatcher`](@ref)
"""
struct ScanMatcherPose2{T} <: IIF.AbstractRelativeMinimize
  """ reference image for scan matching. """
  im1::Matrix{T}
  """ test image to scan match against the reference image. """
  im2::Matrix{T}
  """ Common grid scale for both images -- i.e. units/pixel.  
  Constructor uses two arguments `gridlength`*`rescale=1`=`gridscale`.
  Arg 0 < `rescale` ≤ 1 is also used to rescale the images to lower resolution for speed. """
  gridscale::Float64

  # replace inner constructor with transform on image
  ScanMatcherPose2{T}( im1::AbstractMatrix{T}, 
                      im2::AbstractMatrix{T},
                      gridlength::Real,
                      rescale::Real=1,
                      cvt = (im)->reverse(Images.imresize(im,trunc.(Int, rescale.*size(im))),dims=1)
                    ) where {T} = new{T}( cvt(im1), 
                                          cvt(im2),
                                          rescale*gridlength )
end



struct PackedScanMatcherPose2 <: AbstractPackedFactor
  im1::Vector{Vector{Float64}}
  im2::Vector{Vector{Float64}}
  gridscale::Float64
end