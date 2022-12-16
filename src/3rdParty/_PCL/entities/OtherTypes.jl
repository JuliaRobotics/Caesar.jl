
abstract type AbstractBoundingBox end

Base.@kwdef struct AxisAlignedBoundingBox <: AbstractBoundingBox
  """ axis aligned bounding box, leveraging GeometryBasics.Rect3 (towards GoeB.HyperRectangle) """
  hr::GeoB.Rect3{<:Any} = GeoB.Rect3(GeoB.Point(0,0,0.), GeoB.Point(1,1,1.))
end

function AxisAlignedBoundingBox(
  org::Union{<:AbstractVector{<:Real}, <:GeoB.Point},
  wdt::Union{<:AbstractVector{<:Real}, <:GeoB.Point},
)
  AxisAlignedBoundingBox( 
    hr = GeoB.Rect3(
      GeoB.Point(org...), 
      GeoB.Point(wdt...)
    )
  )
end

function Base.getproperty(aabb::AxisAlignedBoundingBox, f::Symbol)
  if f == :hr
    getfield(aabb, f)
  elseif f == :origin
    aabb.hr.origin
  elseif f == :widths
    aabb.hr.widths
  else
    error("AxisAlignedBoundingBox has no field $f")
  end
end


Base.@kwdef struct OrientedBoundingBox <: AbstractBoundingBox
  """ axis aligned bounding box, leveraging GeometryBasics.Rect3 (towards GoeB.HyperRectangle) """
  hr::GeoB.Rect3{<:Any} = GeoB.Rect3(GeoB.Point(0,0,0.), GeoB.Point(1,1,1.))
  """ Inverse homography which goes from some reference frame r to the bounding box frame """
  bb_H_r::typeof(SMatrix{4,4}(diagm(ones(4)))) = SMatrix{4,4}(diagm(ones(4)))
end

function OrientedBoundingBox(
  org::Union{<:AbstractVector{<:Real}, <:GeoB.Point},
  wdt::Union{<:AbstractVector{<:Real}, <:GeoB.Point},
  position::Union{<:AbstractVector{<:Real}, <:GeoB.Point},
  rotation::AbstractMatrix{<:Real}
)
  r_H_bb = affine_matrix(SpecialEuclidean(3), ArrayPartition(position, rotation))
  OrientedBoundingBox(;
    hr     = GeoB.Rect3(GeoB.Point(org...), GeoB.Point(wdt...)),
    bb_H_r = inv(SMatrix{4,4}(r_H_bb))
  )
end

function Base.getproperty(obb::OrientedBoundingBox, f::Symbol)
  if f == :hr
    getfield(obb, f)
  elseif f == :bb_H_r
    getfield(obb, f)
  elseif f == :origin
    obb.hr.origin
  elseif f == :widths
    obb.hr.widths
  elseif f == :position
    inv(obb.bb_H_r)[1:end-1, end]
  elseif f == :rotation
    inv(obb.bb_H_r)[1:end-1,1:end-1]
  else
    error("OrientedBoundingBox has no field $f")
  end
end