# conversions between ROS and _PCL

@info "Caesar._PCL is loading tools related to RobotOS.jl."
@debug "Caesar._PCL RobotOS.jl requires user to first run as @rostypegen (on messages below) in `Main` before loading Caesar."
@debug "  @rosimport sensor_msgs.msg: PointCloud2" 

using ..RobotOS

@rosimport sensor_msgs.msg: PointCloud2


Base.convert(::Type{UInt64}, rost::RobotOS.Time) = UInt64(rost.secs)*1000000 + trunc(UInt64, rost.nsecs*1e-3)

# Really odd constructor, strong assumption that user FIRST ran @rostypegen in Main BEFORE loading Caesar
function PCLPointCloud2(pc2::Main.sensor_msgs.msg.PointCloud2)
  @error("work in progress on PCLPointCloud2(pc2::Main.sensor_msgs.msg.PointCloud2)")
  header = Header(;stamp = pc2.header.stamp,
                  seq    = pc2.header.seq,
                  frame_id= pc2.header.frame_id )
  #

  # all PointField elements
  pfs = PointField[PointField(;name=pf_.name,offset=pf_.offset,datatype=pf_.datatype,count=pf_.count) for pf_ in pc2.fields]

  PCLPointCloud2(;header,
                  height     = pc2.height, 
                  width      = pc2.width,
                  fields     = pfs,
                  point_step = pc2.point_step,
                  row_step   = pc2.row_step )
                  # is_dense   = pc2.is_dense )
end