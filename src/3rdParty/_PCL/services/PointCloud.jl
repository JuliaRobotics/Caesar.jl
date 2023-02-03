
Base.sizeof(pt::PointXYZ) = sizeof(pt.data)

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

## ==============================================================================================
## translating property names from upside-down C++ meta functions not clean Julian style yet, but 
## can easily refactor once enough unit tests for conversions and use-cases exist here in Julia

function Base.hasproperty(P::Type{<:PointXYZ}, f::Symbol)
  # https://github.com/PointCloudLibrary/pcl/blob/35e03cec65fb3857c1d4062e4bf846d841fb98df/common/include/pcl/conversions.h#L126
  # https://github.com/PointCloudLibrary/pcl/blob/35e03cec65fb3857c1d4062e4bf846d841fb98df/common/include/pcl/for_each_type.h#L70-L86
  # TODO, missing colors when point has RGB
  f in union(fieldnames(P), [:x;:y;:z])
end

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

Base.length(pc::PointCloud) = length(pc.points)

## not very Julian translations from C++ above
## ==============================================================================================


# builds a new immutable object, reuse=true will modify and reuse parts of A and B
function Base.cat(A::PointCloud, B::PointCloud; reuse::Bool=false, stamp_earliest::Bool=true)
  pc = PointCloud(;
    header = Header(;
      seq = A.header.seq,
      stamp = stamp_earliest ? A.header.stamp : maximum(A.header.stamp, B.header.stamp),
      frame_id = A.header.frame_id
    ),
    # can go a little faster, but modifies A
    points = reuse ? A.points : deepcopy(A.points),
    height = A.height,
    width = A.width + B.width,
    is_dense = A.is_dense && B.is_dense
  )
  lenA = length(A.points)
  lenB = length(B.points)
  resize!(pc.points, lenA+lenB)
  pc.points[(lenA+1):end] .= B.points

  # return the new PCLPointCloud2 object
  return pc
end

##

function (fm::FieldMatches{PointXYZ{C,T}})(field::PointField) where {C,T}
  # https://github.com/PointCloudLibrary/pcl/blob/35e03cec65fb3857c1d4062e4bf846d841fb98df/common/include/pcl/PCLPointField.h#L57
  # TODO complete all tests
  hasproperty(PointXYZ{C,T}, Symbol(field.name)) && 
  asType{field.datatype}() == T && 
  (field.count == 1 || # FIXME, SHOULD NOT HARD CODE 1
  field.count == 0 && false) #.size == 1)
    # ((field.count == traits::datatype<PointT, Tag>::size) ||
    # (field.count == 0 && traits::datatype<PointT, Tag>::size == 1 /* see bug #821 */)));
end

# https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00091
# https://github.com/PointCloudLibrary/pcl/blob/903f8b30065866ae5ca57f4c3606437476b51fcc/common/include/pcl/point_traits.h
function (fm!::FieldMapper{T})() where T
  for field in fm!.fields_
    if FieldMatches{T}()(field)
      mapping = FieldMapping(;
        serialized_offset = field.offset,
        struct_offset     = field.offset, # FIXME which offset value to use here ???
        size              = sizeof(asType{field.datatype}())
      )
      push!(fm!.map_, mapping)
      # return nothing
    end
  end
  if 0 < length(fm!.map_)
    return nothing
  end
  @warn "Failed to find match for field..."
end


"""
    $SIGNATURES

Still incomplete, basic 2D and 3D *should* work.

DevNotes
- Resolve, if necessary, conversions endianness with `htol`, `ltoh`, etc.

Notes
- https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00115
- fieldOrdering(a::FieldMapping, b::FieldMapping) = a.serialized_offset < b.serialized_offset
- https://docs.ros.org/en/hydro/api/pcl/html/conversions_8h_source.html#l00123
- https://docs.ros.org/en/jade/api/pcl_conversions/html/namespacepcl.html
"""
function createMapping(T,msg_fields::AbstractVector{<:PointField}, field_map::MsgFieldMap=MsgFieldMap())
  # Create initial 1-1 mapping between serialized data segments and struct fields
  mapper! = FieldMapper{T}(;fields_=msg_fields, map_=field_map)
  # the idea here is that for_each_type<> will recursively call the operator `mapper()` on each element on msg_fields
  # and then check if the desired destination PointCloud{<:PointT}'s points have field names that can absorb
  # the data contained in msg_fields, e.g. `.name==:x`, etc as checked by `FieldMatcher`.  This should
  # add all msg_fields that match properties of destination cloud <:PointT into `field_map`.
  # https://github.com/PointCloudLibrary/pcl/blob/35e03cec65fb3857c1d4062e4bf846d841fb98df/common/include/pcl/conversions.h#L126
  # https://github.com/PointCloudLibrary/pcl/blob/35e03cec65fb3857c1d4062e4bf846d841fb98df/common/include/pcl/for_each_type.h#L70-L86
  # 00127     for_each_type< typename traits::fieldList<PointT>::type > (mapper);
  mapper!()

  # Coalesce adjacent fields into single copy where possible
  if 1 < length(field_map)
    # TODO check accending vs descending order>
    sort!(field_map, by = x->x.serialized_offset)
    
    # something strange with how C++ does and skips the while loop, disabling the coalescing for now
    # i = 1
    # j = i + 1
    # # TODO consolidate strips of memory into a single field_map -- e.g. [x,y,z],offsets=0,size=4 becomes, [x],offsets=0,size=12 
    # _jend = field_map[end]
    # _jmap = field_map[j]
    # while _jmap != _jend
    #   # This check is designed to permit padding between adjacent fields.
    #   if (_jmap.serialized_offset - field_map[i].serialized_offset) == (_jmap.struct_offset - field_map[i].struct_offset)
    #     field_map[i].size += (_jmap.struct_offset + _jmap.size) - (field_map[i].struct_offset + field_map[i].size)
    #     @info "deleteat j" j
    #     # https://www.cplusplus.com/reference/vector/vector/erase/
    #     deleteat!(field_map,j)
    #     _jmap = field_map[j] # same iterator j now points to shifted element in vector after deletion (still the same position after i). 
    #     _jend = field_map[end]
    #   else
    #     i += 1
    #     j += 1
    
    #     _jmap = field_map[j]
    #     _jend = field_map[end]
    #   end
    # end
    # @info "after coalesce" field_map
  end

  return field_map
end
# pc2.fields
#   6-element Vector{Caesar._PCL.PointField}:
#  Caesar._PCL.PointField("x", 0x00000000, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("y", 0x00000004, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("z", 0x00000008, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("intensity", 0x00000010, Caesar._PCL._PCL_FLOAT32, 0x00000001)
#  Caesar._PCL.PointField("timestamp", 0x00000018, Caesar._PCL._PCL_FLOAT64, 0x00000001)
#  Caesar._PCL.PointField("ring", 0x00000020, Caesar._PCL._PCL_UINT16, 0x00000001)


_idxcolor(color::Colorant, i::Integer) = color
_idxcolor(color::AbstractVector{<:Colorant}, i::Integer) = color[i]


function PointCloud(
    x::AbstractVector{<:Real},
    y::AbstractVector{<:Real},
    z::AbstractVector{<:Real};
    color::Union{C, <:AbstractVector{C}} = RGBA(1,1,1,1),
    pointcloudkws...
  ) where {C <: Colorant}
  #

  height = UInt32(1)
  width = UInt32(length(x))

  pt = PointXYZ(;data=SA[x[1];y[1];z[1];1])
  points = Vector{typeof(pt)}(undef, width)
  points[1] = pt
  for i in 2:width
    points[i] = PointXYZ(;data=SA[x[i];y[i];z[i];1], color=_idxcolor(color, i))
  end

  PointCloud(; height, width, points, pointcloudkws...)
end

function PointCloud(
  xyz::AbstractMatrix{<:Real};
  kwargs...
)
  #
  @cast mat[i,d] := xyz[i][d]
  PointCloud(
    mat;
    kwargs...
  )
end

function PointCloud(
  xyz::AbstractVector{<:AbstractVector{<:Real}};
  kwargs...
)



# https://pointclouds.org/documentation/conversions_8h_source.html#l00166
function PointCloud(
    msg::PCLPointCloud2, 
    cloud::PointCloud{T} = PointCloud(;
      header   = msg.header,
      width    = msg.width,
      height   = msg.height,
      is_dense = msg.is_dense == 1 ),
    field_map::MsgFieldMap=createMapping(T,msg.fields)
  ) where {T}
  #
  cloudsize = msg.width*msg.height
  # cloud_data = Vector{UInt8}(undef, cloudsize)

  # FIXME cannot assume all fields use the same data type
  # off script conversion for XYZ_ data only
  datatype = asType{msg.fields[1].datatype}()
  len = trunc(Int, length(msg.data)/field_map[1].size)
  data_ = Vector{datatype}(undef, len)
  read!(IOBuffer(msg.data), data_)
  mat = reshape(data_, :, cloudsize)
  # see createMapping above
  

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
    @assert msg.height == 1 "only decoding msg::PCLPointCloud2.height=1 messages, update converter here."
    for row in 1:msg.height
      # TODO check might have an off by one error here
      # row_data = row * msg.row_step + 1 # msg.data[(row-1) * msg.row_step]
      for col in 1:msg.width
        # msg_data = row_data + col*msg.point_step
        # the slow way of building the point.data entry
        ptdata = zeros(datatype, 4)
        for (i,mapping) in enumerate(field_map)
          # @info "test" i mapping maxlog=3
          midx = trunc(Int,mapping.serialized_offset/mapping.size) + 1
          # Copy individual points as columns from mat -- i.e. memcpy
          # TODO copy only works if all elements have the same type -- suggestion, ptdata::ArrayPartition instead
          ptdata[i] = mat[midx, col] 
        end
        pt = T(;data=SVector(ptdata...))
        push!(cloud.points, pt)
        # cloudsize += sizeof(T)
      end
    end
  end

  return cloud
end


"""
$SIGNATURES

Convert a pcl::PointCloud<T> object to a PCLPointCloud2 binary data blob.
In: cloud the input pcl::PointCloud<T>
Out: msg the resultant PCLPointCloud2 binary blob

DevNotes:
- TODO, remove hacks, e.g. fields [x,y,z,intensity,]-only.
"""
function PCLPointCloud2(cloud::PointCloud{T,P,R}; datatype = _PCL_FLOAT32) where {T,P,R}
  # Ease the user's burden on specifying width/height for unorganized datasets
  if (cloud.width == 0 && cloud.height == 0)
    height = 1;
    width  = length(cloud)
  else
    @assert (length(cloud) == cloud.width * cloud.height) "Input `cloud::PointCloud`'s width*height is not equal to length(cloud)"
    height = cloud.height;
    width  = cloud.width;
  end

  #    // Fill point cloud binary data (padding and all)
  #    std::size_t data_size = sizeof (PointT) * cloud.size ();
  #    msg.data.resize (data_size);
  #    if (data_size)
  #    {
  #      memcpy(&msg.data[0], &cloud[0], data_size);
  #    }
  
  fields = Vector{PointField}(undef, 4)
  # TODO assuming all(fields[_].count==1)
  fields[1] = PointField("x", UInt32(0), datatype, UInt32(1))
  fields[2] = PointField("y", UInt32(4), datatype, UInt32(1))
  fields[3] = PointField("z", UInt32(8), datatype, UInt32(1))
  fields[4] = PointField("intensity", UInt32(12), datatype, UInt32(1))
  _nflds = length(fields)
  #    # Fill fields metadata
  #    msg.fields.clear ();
  #    for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));
  
  # NOTE get the last possible byte location of all fields, assume no padding after last used byte
  _fend = fields[end]
  point_step = _fend.offset + sizeof(asType{_fend.datatype}())  # = sizeof (PointT);
  #    msg.row_step   = (sizeof (PointT) * msg.width);
  #    # todo msg.is_bigendian = ?;
  
  # flatten all point fields as binary
  data = Vector{UInt8}(undef, point_step*width)
  for (i,pt) in enumerate(cloud.points)
    offset = (point_step*(i-1)+1)
    # NOTE assume continuous data block in struct (all of same datatype, FIXME)
    data[offset:(offset-1+point_step)] .= reinterpret(UInt8, view(pt.data, 1:_nflds))
  end

  # @error("noise")
  return PCLPointCloud2(;
    header = cloud.header,
    height,
    width,
    fields,
    data,
    # is_bigendian = ,
    point_step,
    row_step = sizeof(T)*width,
    is_dense = cloud.is_dense,
  )
end

## =========================================================================================================
## Useful utils
## =========================================================================================================


function _filterMinRange(
  pts::AbstractVector{<:AbstractVector{<:Real}}, 
  minrange::Real, 
  maxrange::Real
)
  # filter pointcloud section of interest
  msk = findall(x-> minrange < norm(x) < maxrange, pts)
  pts[msk]
end

function _prepPointCloud(
  pc_a::PointCloud,
  dims=1:3;
  minrange=0.0,
  maxrange=999.0
)
  pts_a_ = (s->s.data[dims]).(pc_a.points)
  pts_a = _PCL._filterMinRange(pts_a_, minrange, maxrange)
    # pts_a = (s->[s.x;s.y;s.z]).(pc_a.points)
  # To convert them to a matrix.
  @cast pts_matrix[i,d] := pts_a[i][d]
  return collect(pts_matrix)
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
  println(io)
  printstyled(io, "Caesar._PCL.PCLPointCloud2", bold=true, color=:blue)
  # println(io)
  # printstyled(io, "    T = ", bold=true, color=:magenta)
  # println(io, T)
  # printstyled(io, " }", bold=true, color=:blue)
  println(io)
  print(io, "  header::", pc.header)
  println(io, "  height:       ", pc.height)
  println(io, "  width:        ", pc.width)
  print(io, "  # fields:     ", length(pc.fields))
  if 0 < length(pc.fields)
    print(io, ":  [")
    for fld in pc.fields
      print(io, fld.name, ",")
    end
    print(io, "]")
  end
  println(io)
  println(io, "  # data[]:     ", length(pc.data) )
  println(io, "  is_bigendian: ", pc.is_bigendian)
  println(io, "  point_step:   ", pc.point_step )
  println(io, "  row_step:     ", pc.row_step )
  println(io, "  is_dense:     ", pc.is_dense )
  println(io)
  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::PCLPointCloud2) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::PCLPointCloud2) = show(io, pc)



function Base.show(io::IO, pc::PointCloud{T,P,R}) where {T,P,R}
  println(io)
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
  q = convert(_Rot.QuatRotation, pc.sensor_orientation_)
  println(io, "    w_xyz*: ", round.(_Rot.params(q), digits=3))

  nothing
end

Base.show(io::IO, ::MIME"text/plain", pc::PointCloud) = show(io, pc)
Base.show(io::IO, ::MIME"application/prs.juno.inline", pc::PointCloud) = show(io, pc)




#