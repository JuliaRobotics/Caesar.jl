


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
Base.getindex(pc::PCLPointCloud2, i) = pc.data[i]
Base.setindex!(pc::PCLPointCloud2, pt::PointT, idx) = (pc.data[idx] = pt)
Base.resize!(pc::PCLPointCloud2, s::Integer) = resize!(pc.data, s)

# Add a few basic dispatches
Base.getindex(pc::PointCloud, i) = pc.points[i]
Base.setindex!(pc::PointCloud, pt::PointT, idx) = (pc.points[idx] = pt)
Base.resize!(pc::PointCloud, s::Integer) = resize!(pc.points, s)

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

  # return the new PCLPointCloud2 object
  return pc
end

## https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00123
# 0123   createMapping (const std::vector<pcl::PCLPointField>& msg_fields, MsgFieldMap& field_map)
# 00124   {
# 00125     // Create initial 1-1 mapping between serialized data segments and struct fields
# 00126     detail::FieldMapper<PointT> mapper (msg_fields, field_map);
# 00127     for_each_type< typename traits::fieldList<PointT>::type > (mapper);
# 00128 
# 00129     // Coalesce adjacent fields into single memcpy's where possible
# 00130     if (field_map.size() > 1)
# 00131     {
# 00132       std::sort(field_map.begin(), field_map.end(), detail::fieldOrdering);
# 00133       MsgFieldMap::iterator i = field_map.begin(), j = i + 1;
# 00134       while (j != field_map.end())
# 00135       {
# 00136         // This check is designed to permit padding between adjacent fields.
# 00139         if (j->serialized_offset - i->serialized_offset == j->struct_offset - i->struct_offset)
# 00140         {
# 00141           i->size += (j->struct_offset + j->size) - (i->struct_offset + i->size);
# 00142           j = field_map.erase(j);
# 00143         }
# 00144         else
# 00145         {
# 00146           ++i;
# 00147           ++j;
# 00148         }
# 00149       }
# 00150     }
# 00151   }

function PointCloud(pc2::PCLPointCloud2)

  cloud_data = Vector{UInt8}(undef, pc2.width*pc2.height)

  # Check if we can copy adjacent points in a single memcpy.  We can do so if there
  # is exactly one field to copy and it is the same size as the source and destination
  # point types.
  if (field_map.size() == 1 &&
      field_map[0].serialized_offset == 0 &&
      field_map[0].struct_offset == 0 &&
      field_map[0].size == msg.point_step &&
      field_map[0].size == sizeof(PointT))
    #

  else

  end

  pc = PointCloud(;
    header  = pc2.header,
    # points = pc2.data, # must first convert reference frame
    width   = pc2.width,
    height  = pc2.height,
    is_dense= pc2.is_dense == UInt8(1)
  )
end

## =========================================================================================================
## Custom printing
## =========================================================================================================


function Base.show(io::IO, hdr::Header) where {T}
  printstyled(io, "Caesar._PCL.Header", bold=true, color=:blue)
  println(io)
  println(io, "   seq:       ", hdr.seq)
  println(io, "   stamp*:    ", unix2datetime(hdr.stamp*1e-6))
  println(io, "   frame_id:  ", hdr.frame_id)

  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::Header) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::Header) = show(io, pc)


function Base.show(io::IO, pc::PCLPointCloud2{T}) where {T}
  printstyled(io, "Caesar._PCL.PCLPointCloud2{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    T = ", bold=true, color=:magenta)
  println(io, T)
  printstyled(io, " }", bold=true, color=:blue)
  println(io)
  println(io, "  header::", pc.header)
  println(io, "  height:       ", pc.height)
  println(io, "  width:        ", pc.width)
  println(io, "  data[::T]:    ", length(pc.data) )
  println(io, "  point_step:   ", pc.point_step )
  println(io, "  row_step:     ", pc.row_step )
  println(io, "  is_dense:     ", pc.is_dense)

  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::PCLPointCloud2) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::PCLPointCloud2) = show(io, pc)



function Base.show(io::IO, pc::PointCloud{T,P,R}) where {T,P,R}
  printstyled(io, "Caesar._PCL.PointCloud{", bold=true, color=:blue)
  println(io)
  printstyled(io, "    T = ", bold=true, color=:magenta)
  println(io, T)
  printstyled(io, "    P = ", bold=true, color=:magenta)
  println(io, P)
  printstyled(io, "    R = ", bold=true, color=:magenta)
  println(io, R)
  printstyled(io, " }", bold=true, color=:blue)
  println(io)
  println(io, "  header:       ", pc.header)
  println(io, "  width:        ", pc.width)
  println(io, "  height:       ", pc.height)
  println(io, "  points[::T]:  ", length(pc.points) )
  println(io, "  sensor pose:")
  println(io, "    xyz:    ", round.(pc.sensor_origin_, digits=3))
  q = convert(_Rot.UnitQuaternion, pc.sensor_orientation_)
  println(io, "    w_xyz*: ", round.([q.w; q.x; q.y; q.z], digits=3))

  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::PointCloud) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::PointCloud) = show(io, pc)




#