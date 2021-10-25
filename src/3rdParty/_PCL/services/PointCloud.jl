


# Construct helpers nearest to PCL
PointXYZRGBA( x::Real=0, y::Real=0, z::Real=Float32(1); 
              r::Real=1,g::Real=1,b::Real=1, alpha::Real=1,
              color::Colorant=RGBA(r,g,b,alpha), 
              pos=SA[x,y,z], data=SA[pos...,1] ) = PointXYZ(;color,data)
#
PointXYZRGB(  x::Real=0, y::Real=0, z::Real=Float32(1); 
              r::Real=1,g::Real=1,b::Real=1, 
              color::Colorant=RGB(r,g,b), 
              pos=SA[x,y,z], data=SA[pos...,1] ) = PointXYZ(;color,data)
#

function Base.getproperty(p::PointXYZ, f::Symbol)
  if f == :x 
    return getfield(p, :data)[1]
  elseif f == :y 
    return getfield(p, :data)[2]
  elseif f == :z
    return getfield(p, :data)[3]
  elseif f == :r
    return getfield(p, :color).r
  elseif f == :g
    return getfield(p, :color).g
  elseif f == :b
    return getfield(p, :color).b
  elseif f == :alpha
    return getfield(p, :color).alpha
  elseif f == :data || f == :pos
    return getfield(p, :data)
  elseif f == :color || f == :rgb
    return getfield(p, :color)
  end
  error("PointXYZ has no field $f")
end



# Add a few basic dispatches
Base.getindex(pc::PointCloud, i) = pc.data[i]
Base.setindex!(pc::PointCloud, pt::PointT, idx) = (pc.data[idx] = pt)
Base.resize!(pc::PointCloud, s::Integer) = resize!(pc.data, s)

# builds a new immutable object, reuse=true will modify and reuse parts of A and B
function Base.cat(A::PointCloud, B::PointCloud; reuse::Bool=false)
  pc = PointCloud(;
    header = PCLHeader(;
      seq = A.header.seq,
      stamp = maximum(A.header.stamp, B.header.stamp),
      frame_id = A.header.frame_id
    ),
    # can go a little faster, but modifies A
    points = reuse ? A.points : deepcopy(A.points),
    width = A.width,
    height = 1,
    is_dense = A.is_dense && B.is_dense
  )
  lenA = length(A.points)
  lenB = length(B.points)
  resize!(pc.points, lenA+lenB)
  pc.points[(lenA+1):end] .= B.points

  # return the new PointCloud object
  return pc
end