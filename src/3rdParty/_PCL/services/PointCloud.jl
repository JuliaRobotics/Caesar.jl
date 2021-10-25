


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
    points = reuse ? A.data : deepcopy(A.data),
    width = A.width,
    height = 1,
    is_dense = A.is_dense && B.is_dense
  )
  lenA = length(A.data)
  lenB = length(B.data)
  resize!(pc.data, lenA+lenB)
  pc.data[(lenA+1):end] .= B.data

  # return the new PointCloud object
  return pc
end


function Base.show(io::IO, pc::PointCloud{T}) where {T}
  printstyled(io, "Caesar._PCL.PointCloud{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    T = ", bold=true, color=:magenta)
  println(io, T)
  # part of version 1 data type
  # printstyled(io, "    P = ", bold=true, color=:magenta)
  # println(io, P)
  # printstyled(io, "    R = ", bold=true, color=:magenta)
  # println(io, R)
  printstyled(io, " }", bold=true, color=:blue)
  println(io)
  println(io, "  header:       ", pc.header)
  println(io, "  height:       ", pc.height)
  println(io, "  width:        ", pc.width)
  println(io, "  data[::T]:    ", length(pc.data) )
  println(io, "  point_step:   ", pc.point_step )
  println(io, "  row_step:     ", pc.row_step )
  println(io, "  is_dense:     ", pc.is_dense)
  # part of version 1 data type
  # println(io, "  sensor pose:")
  # println(io, "    xyz:    ", round.(pc.sensor_origin_, digits=3))
  # q = convert(_Rot.UnitQuaternion, pc.sensor_orientation_)
  # println(io, "    w_xyz*: ", round.([q.w; q.x; q.y; q.z], digits=3))

  nothing
end

Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::PointCloud) = show(io, pc)


#