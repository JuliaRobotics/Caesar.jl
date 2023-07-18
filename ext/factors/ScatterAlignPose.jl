

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
