# support saving and loading PointClouds as las files

@info "Caesar._PCL is loading features using LasIO.jl"

using .LasIO
using .FileIO
using .Dates
using .TensorCast


function loadLAS(
  filepath::AbstractString
)
  header, points = load(filepath)
  pts = (p->[
    p.x*header.x_scale+header.x_min; 
    p.y*header.y_scale+header.y_min; 
    p.z*header.z_scale+header.z_min
  ]).(points)
  @cast pts_[i,d] := pts[i][d]
  return PointCloud(pts_)
end


# TODO, make save LAS
function saveLAS(
  filepath::AbstractString,
  pc::PointCloud,
)
  
  # pts = (s->s.data[1:3]).(pc.points)
  x = map(p->p.data[1], pc.points)
  y = map(p->p.data[2], pc.points)
  z = map(p->p.data[3], pc.points)
  pCloud = LasPoint2.(round.(1000*x),round.(1000*y),round.(1000*z), 1, 0, 0, 0, 0, 0, 1, 1, 1);

  tim = pc.header.stamp # now()

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
  x_scale = 0.001
  y_scale = 0.001
  z_scale = 0.001
  x_offset = 0.0
  y_offset = 0.0
  z_offset = 0.0
  x_max = maximum(x) # 7.6900000
  x_min = minimum(x) # -12.8500000
  y_max = maximum(y) # 9.9200000
  y_min = minimum(y) # -10.1800000
  z_max = maximum(z) # 2.3300000
  z_min = minimum(z) # 1.8400000

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