
## FIXME DOUBLCHECK
Base.sizeof(::Type{PointXYZ{C,T}}) where {C,T} = sizeof(T)

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

# https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00115
# fieldOrdering(a::FieldMapping, b::FieldMapping) = a.serialized_offset < b.serialized_offset

# https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00123
# https://docs.ros.org/en/jade/api/pcl_conversions/html/namespacepcl.html
function createMapping(msg_fields::AbstractVector{<:PointField}, field_map::MsgFieldMap=MsgFieldMap())
  # Create initial 1-1 mapping between serialized data segments and struct fields
  mapper = FieldMapper(;fields_=msg_fields, map_=field_map)
  # 00127     for_each_type< typename traits::fieldList<PointT>::type > (mapper);

  # Coalesce adjacent fields into single copy where possible
  if 1 < length(field_map)
    # TODO check accending vs descending order
    sort!(field_map, by = x->x.serialized_offset)
    i = 1
    j = i + 1
    # FIXME WIP ...[j] != field_map[end]
    while j <= length(field_map)
      # This check is designed to permit padding between adjacent fields.
      if (field_map[j].serialized_offset - field_map[i].serialized_offset) == (field_map[j].struct_offset - field_map[i].struct_offset)
        field_map[i].size += (field_map[j].struct_offset + field_map[j].size) - (field_map[i].struct_offset + field_map[i].size)
        # https://www.cplusplus.com/reference/vector/vector/erase/
        deleteat!(field_map,j)
        j += 1
        # j = field_map.erase(j);
      else
        i += 1
        j += 1
      end
    end
  end

  return field_map
end


# https://pointclouds.org/documentation/conversions_8h_source.html#l00166
function PointCloud(
    msg::PCLPointCloud2, 
    cloud::PointCloud{T} = PointCloud(;
      header   = msg.header,
      width    = msg.width,
      height   = msg.height,
      is_dense = msg.is_dense == 1 ),
    field_map::MsgFieldMap=createMapping(msg.fields)
  ) where {T}
  #
  cloudsize = msg.width*msg.height
  # cloud_data = Vector{UInt8}(undef, cloudsize)

  @info "field_map" length(field_map)

  # Check if we can copy adjacent points in a single memcpy.  We can do so if there
  # is exactly one field to copy and it is the same size as the source and destination
  # point types.
  if (length(field_map) == 1 &&
      field_map[1].serialized_offset == 0 &&
      field_map[1].struct_offset == 0 &&
      field_map[1].size == msg.point_step &&
      field_map[1].size == sizeof(T)) 
    #
    error("copy of just one field_map not implemented yet")
  else
    # If not, memcpy each group of contiguous fields separately
    @info "not just a copy of one field_map" Int(msg.height) Int(msg.width) sizeof(T)
    for row in 0:(msg.height-1)
      # @show Int(msg.row_step)
      # TODO check might have an off by one error here
      @show row_data = row * msg.row_step + 1 # msg.data[(row-1) * msg.row_step]
      for col in 0:(msg.width-1)
        @show msg_data = row_data + col*msg.point_step
        for mapping in field_map
          # memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
          @info "copy" mapping.struct_offset mapping.serialized_offset mapping.size
        end
        cloudsize += sizeof(T)
      end
    end
  end

  return cloud
end

## =========================================================================================================
## Custom printing
## =========================================================================================================


function Base.show(io::IO, hdr::Header) # where {T}
  printstyled(io, "Caesar._PCL.Header", bold=true, color=:blue)
  println(io)
  println(io, "   seq:       ", hdr.seq)
  println(io, "   stamp*:    ", unix2datetime(hdr.stamp*1e-6))
  println(io, "   frame_id:  ", hdr.frame_id)

  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::Header) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::Header) = show(io, pc)


function Base.show(io::IO, pc::PCLPointCloud2)
  printstyled(io, "Caesar._PCL.PCLPointCloud2", bold=true, color=:blue)
  println(io)
  # printstyled(io, "    T = ", bold=true, color=:magenta)
  # println(io, T)
  # printstyled(io, " }", bold=true, color=:blue)
  println(io)
  println(io, "  header::", pc.header)
  println(io, "  height:       ", pc.height)
  println(io, "  width:        ", pc.width)
  println(io, "  # fields:     ", length(pc.fields))
  println(io, "  # data[]:     ", length(pc.data) )
  println(io, "  is_bigendian: ", pc.is_bigendian)
  println(io, "  point_step:   ", pc.point_step )
  println(io, "  row_step:     ", pc.row_step )
  println(io, "  is_dense:     ", pc.is_dense )
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
  println(io, "  is_dense:     ", pc.is_dense)
  println(io, "  sensor pose:")
  println(io, "    xyz:    ", round.(pc.sensor_origin_, digits=3))
  q = convert(_Rot.UnitQuaternion, pc.sensor_orientation_)
  println(io, "    w_xyz*: ", round.([q.w; q.x; q.y; q.z], digits=3))

  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::PointCloud) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::PointCloud) = show(io, pc)




#