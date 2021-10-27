# conversions between ROS and _PCL

@info "Caesar._PCL is loading tools related to RobotOS.jl."
@debug "Caesar._PCL RobotOS.jl requires user to first run as @rostypegen (on messages below) in `Main` before loading Caesar."
@debug "  @rosimport sensor_msgs.msg: PointCloud2" 

using ..RobotOS

@rosimport sensor_msgs.msg: PointCloud2


Base.convert(::Type{UInt64}, rost::RobotOS.Time) = UInt64(rost.secs)*1000000 + trunc(UInt64, rost.nsecs*1e-3)

# Really odd constructor, strong assumption that user FIRST ran @rostypegen in Main BEFORE loading Caesar
# https://docs.ros.org/en/hydro/api/pcl_conversions/html/pcl__conversions_8h_source.html#l00208
function PCLPointCloud2(msg::Main.sensor_msgs.msg.PointCloud2)
  @warn("work in progress on PCLPointCloud2(msg::Main.sensor_msgs.msg.PointCloud2)")
  header = Header(;stamp  = msg.header.stamp,
                  seq     = msg.header.seq,
                  frame_id= msg.header.frame_id )
  #

  # all PointField elements
  pfs = PointField[PointField(;name=pf_.name,offset=pf_.offset,datatype=pf_.datatype,count=pf_.count) for pf_ in msg.fields]

  endian = msg.is_bigendian ? _PCL_ENDIAN_BIG_BYTE : _PCL_ENDIAN_LITTLE_WORD
  @show typeof(msg.data) length(msg.data)
  pc2 = PCLPointCloud2(;header,
                        height     = msg.height, 
                        width      = msg.width,
                        fields     = pfs,
                        data       = msg.data,
                        is_bigendian= endian,
                        point_step = msg.point_step,
                        row_step   = msg.row_step,
                        is_dense   = UInt8(msg.is_dense) )
end

#