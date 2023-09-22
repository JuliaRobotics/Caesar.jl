# support saving and loading PointClouds as las files

# @info "Caesar._PCL is loading features using LasIO.jl"

# using .LasIO
# using .FileIO
# using .Dates
# using .TensorCast


function loadLAS(
  filepath::Union{<:AbstractString, <:Stream}
)
  header, points = load(filepath)
  pts = map(points) do p
    SA[
      p.x * header.x_scale + header.x_offset; 
      p.y * header.y_scale + header.y_offset; 
      p.z * header.z_scale + header.z_offset; 
      1
    ]
  end

  colors = map(points) do pnt
    RGB(pnt.red, pnt.green, pnt.blue)
  end

  colored_points = _PCL.PointXYZ.(colors, pts)
  return _PCL.PointCloud(;points=colored_points, width=UInt32(length(colored_points)), height=UInt32(1))
end


# TODO, make save LAS
function saveLAS(
  filepath::Union{<:AbstractString, <:Stream},
  pc::_PCL.PointCloud;
  scale=1000
)
  
  # pts = (s->s.data[1:3]).(pc.points)
  x = map(p->p.data[1], pc.points)
  y = map(p->p.data[2], pc.points)
  z = map(p->p.data[3], pc.points)
  r = map(p->p.color.r, pc.points)
  g = map(p->p.color.g, pc.points)
  b = map(p->p.color.b, pc.points)
  pCloud = LasPoint2.(round.(scale*x),round.(scale*y),round.(scale*z), 1, 0, 0, 0, 0, 0, r, g, b);

  tim = unix2datetime(pc.header.stamp*1e-6) # now()

  file_source_id = 0
  global_encoding = 0
  guid_1 = 0
  guid_2 = 0
  guid_3 = 0
  guid_4 = ""
  version_major = 1
  version_minor = 2
  system_id = "OTHER"
  software_id = "CJL v0.14.0"
  creation_dayofyear = dayofyear(tim)
  creation_year = year(now())
  header_size = 227
  data_offset = 227
  n_vlr = 0
  data_format_id = 2
  data_record_length = 26
  records_count = length(pc.points)
  point_return_count = UInt32[0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000]
  x_scale = 1/scale
  y_scale = 1/scale
  z_scale = 1/scale
  x_offset = 0.0
  y_offset = 0.0
  z_offset = 0.0
  x_max = maximum(x)+1 # 7.6900000
  x_min = minimum(x)-1 # -12.8500000
  y_max = maximum(y)+1 # 9.9200000
  y_min = minimum(y)-1 # -10.1800000
  z_max = maximum(z)+1 # 2.3300000
  z_min = minimum(z)-1 # 1.8400000

  # newHeader = deepcopy(header)
  newHeader = LasHeader(
    file_source_id,
    global_encoding,
    guid_1,
    guid_2,
    guid_3,
    guid_4,
    version_major,
    version_minor,
    system_id,
    software_id,
    creation_dayofyear,
    creation_year,
    header_size,
    data_offset,
    n_vlr,
    data_format_id,
    data_record_length,
    records_count,
    point_return_count,
    x_scale,
    y_scale,
    z_scale,
    x_offset,
    y_offset,
    z_offset,
    x_max,
    x_min,
    y_max,
    y_min,
    z_max,
    z_min,
    LasVariableLengthRecord[],
    UInt8[]
  )

  save(filepath, newHeader, pCloud)
end
# possibly v1.4 header
# { 
#   FileSignature: 'LASF',
#   FileSoureceID: 0,
#   GlobalEncoding: 0,
#   VersionMajor: 1,
#   VersionMinor: 2,
#   SystemIdentifier: 'PDAL',
#   GeneratingSoftware: 'PDAL 1.5.0 (424c25)',
#   CreationDay: 50,
#   CreationYear: 2018,
#   HeaderSize: 227,
#   OffsetToPointData: 621,
#   NumberOfVariableLengthRecords: 4,
#   PointDataFormatID: 131,
#   PointDataRecordLength: 34,
#   NumberOfPoints: 24318764,
#   NumberOfPointByReturn: 0,
#   ScaleFactorX: 0.001,
#   ScaleFactorY: 0.001,
#   ScaleFactorZ: 0.001,
#   OffsetX: 475000,
#   OffsetY: 7212000,
#   OffsetZ: 0,
#   MaxX: 475111.573,
#   MinX: 474773.203,
#   MaxY: 7212849.161,
#   MinY: 7212550.753,
#   MaxZ: 38.608000000000004,
#   MinZ: -195.541,
#   epsg: 3133 
# }

#