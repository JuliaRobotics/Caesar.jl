# Utilities for scan matching

export overlayScanMatcher


# this function uses the sum of squared differences between the two images.
# To use low-passed versions of the images, simpy set the kernel argument to
# Kernel.gaussian(10) (or an appropriate size)
getMismatch(a,b) = sqrt(sum((a.-b).^2))

# sqrt(sum( imfilter( a.-b, Kernel.gaussian(5)).^2 ))
# sqrt(sum((imfilter(a, kernel).-imfilter(b, kernel)).^2))

# also do MMD version


# Next step is to define a function that applies a transform to the image. This
# transform consists of a translation and a rotation
function transformImage_SE2(img::AbstractMatrix, 
                            tf::Union{<:Manifolds.ProductRepr, <:Manifolds.ArrayPartition},
                            gridscale::Real=1  )
    #
    tf_ = LinearMap(tf.parts[2])∘Translation( gridscale.*tf.parts[1]... )
    tf_img = warp(img, tf_, degree=ImageTransformations.Constant())

    # replace NaN w/ 0
    mask = findall(x->isnan(x), tf_img)
    tf_img[mask] .= 0.0
    return tf_img
end


# now we can combine the two into an evaluation function
function evaluateTransform( a::AbstractMatrix, 
                            b::AbstractMatrix, 
                            tf::Union{<:Manifolds.ProductRepr, <:Manifolds.ArrayPartition},
                            gridscale::Real=1 )
    #
    # transform image
    bp = transformImage_SE2(b,tf, gridscale)

    # get matching padded views
    ap, bpp = paddedviews(0.0, a, bp)
    return getMismatch(ap,bpp)
end


"""
    $SIGNATURES

Overlay the two images from `AlignRadarPose2` with the first (red) fixed and transform the second image (blue) according to `tf`.

Notes:
- `tf` is a Manifolds.jl type `::ProductRepr` (or newer `::ArrayPartition`) to represent a `SpecialEuclidean(2)` manifold point.
"""
function overlayScanMatcher(sm::AlignRadarPose2, 
                            tf = Manifolds.identity_element(SpecialEuclidean(2)) )
    #
    im2_ = transformImage_SE2(sm.im2, tf)
    
    # get matching padded views
    im1_, im2__ = paddedviews(0.0, sm.im1, im2_)

    im1__ = RGBA.(im1_, 0, 0, 0.5)
    im2___ = RGBA.(0, 0, im2__, 0.5)    

    # im2__ = RGBA.(im2_, 0, 0, 0.5)
    im1__ .+ im2___
end

