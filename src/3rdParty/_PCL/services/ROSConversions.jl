# conversions between ROS and _PCL

@info "Caesar._PCL is loading tools related to RobotOS.jl."
@debug "Caesar._PCL RobotOS.jl requires user to first run as @rostypegen (on messages below) in `Main` before loading Caesar."
@debug "  @rosimport sensor_msgs.msg: PointCloud2" 

using ..RobotOS

@rosimport std_msgs.msg: Header
@rosimport sensor_msgs.msg: PointCloud2



Base.convert(::Type{UInt64}, rost::RobotOS.Time) = UInt64(rost.secs)*1000000 + trunc(UInt64, rost.nsecs*1e-3)
# FIXME, this converter is making a small mistake, e.g. the loss of 145ns in comment data below
Base.convert(::Type{RobotOS.Time}, tm::UInt64) = RobotOS.Time(trunc(Int32,tm*1e-6), + trunc(Int32, ((tm*1e-6) % 1)*1_000_000_000) )
# header: 
#   seq: 24282
#   stamp: 
#     secs: 1646305718
#     nsecs: 997857000
#   frame_id: "PandarXT-32"
##
# julia> rt = RobotOS.Time(1646305718, 997857000)
# Time(1646305718, 997857000)
# julia> xt = convert(UInt64, rt)
# 0x0005d94e6b929361
# julia> convert(RobotOS.Time, xt)
# Time(1646305718, 997856855)

# @assert convert(RobotOS.Time, convert(UInt64, RobotOS.Time(1646305718, 997857000))) == RobotOS.Time(1646305718, 997857000) "conversion to and from RobotOS.Time and UInt64 not consistent"

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

# function toROSPointCloud2(pc2::PCLPointCloud2)
#   msg = Main.sensor_msgs.msg.PointCloud2()
#   msg.height = pc2.height

#   msg
# end

#