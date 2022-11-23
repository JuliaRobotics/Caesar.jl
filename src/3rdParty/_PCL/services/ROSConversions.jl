# conversions between ROS and _PCL

@info "Caesar._PCL is loading tools related to RobotOS.jl."
@debug "Caesar._PCL RobotOS.jl requires user to first run as @rostypegen (on messages below) in `Main` before loading Caesar."
@debug "  @rosimport sensor_msgs.msg: PointCloud2" 

using .RobotOS

@rosimport std_msgs.msg: Header
@rosimport sensor_msgs.msg: PointField
@rosimport sensor_msgs.msg: PointCloud2


Base.convert(::Type{UInt64}, rost::RobotOS.Time) = UInt64(rost.secs)*1000000 + trunc(UInt64, rost.nsecs*1e-3)
Base.convert(::Type{RobotOS.Time}, tm::UInt64) = RobotOS.Time(trunc(Int32,tm*1e-6), Int32((tm % 1_000_000)*1000) )

# embedded test to avoid CI requiring all of ROS and PyCall
@assert convert(RobotOS.Time, convert(UInt64, RobotOS.Time(1646305718, 997857000))) == RobotOS.Time(1646305718, 997857000) "conversion to and from RobotOS.Time and UInt64 not consistent"

# Really odd constructor, strong assumption that user FIRST ran @rostypegen in Main BEFORE loading Caesar
# https://docs.ros.org/en/hydro/api/pcl_conversions/html/pcl__conversions_8h_source.html#l00208
function PCLPointCloud2(msg::Main.sensor_msgs.msg.PointCloud2)
  header = Header(;stamp  = msg.header.stamp,
                  seq     = msg.header.seq,
                  frame_id= msg.header.frame_id )
  #

  # all PointField elements
  pfs = PointField[PointField(;name=pf_.name,offset=pf_.offset,datatype=pf_.datatype,count=pf_.count) for pf_ in msg.fields]

  endian = msg.is_bigendian ? _PCL_ENDIAN_BIG_BYTE : _PCL_ENDIAN_LITTLE_WORD
  PCLPointCloud2(;header,
                  height     = msg.height, 
                  width      = msg.width,
                  fields     = pfs,
                  data       = msg.data,
                  is_bigendian= endian,
                  point_step = msg.point_step,
                  row_step   = msg.row_step,
                  is_dense   = UInt8(msg.is_dense) 
  )
  #
end


"""
    $SIGNATURES

Convert `PCLPointCloud2` type to ROS message `sensor_msgs.msg.PointCloud2`.

See also: [`Caesar._PCL.PCLPointCloud2`](@ref), [`Caesar._PCL.PointCloud`](@ref).
"""
function toROSPointCloud2(pc2::PCLPointCloud2)
  header = Main.std_msgs.msg.Header();
  header.seq = pc2.header.seq
  header.stamp = convert(RobotOS.Time, pc2.header.stamp)
  header.frame_id = pc2.header.frame_id

  msg = Main.sensor_msgs.msg.PointCloud2();

  msg.header = header
  msg.height = pc2.height
  msg.width = pc2.width

  # all PointField elements
  fields = Main.sensor_msgs.msg.PointField[]
  for pf_ in pc2.fields
    mpf = Main.sensor_msgs.msg.PointField()
    mpf.name = pf_.name
    mpf.offset=pf_.offset
    mpf.datatype= convert(UInt8, pf_.datatype)
    mpf.count=pf_.count
    push!(fields, mpf)
  end
  msg.fields = fields

  msg.is_bigendian = pc2.is_bigendian == _PCL_ENDIAN_BIG_BYTE
  msg.point_step = pc2.point_step
  msg.row_step = pc2.row_step
  msg.data = pc2.data
  msg.is_dense = pc2.is_dense

  msg
end

#