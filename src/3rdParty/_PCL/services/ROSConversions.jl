# conversions between ROS and _PCL

@info "Caesar._PCL is loading tools related to RobotOS.jl."
@debug "Caesar._PCL RobotOS.jl requires user to first run in `Main` (before loading Caesar)" "@rosimport sensor_msgs.msg: PointCloud2" "@rostypegen" 

using ..RobotOS

@rosimport sensor_msgs.msg: PointCloud2

# Really odd constructor, strong assumption that user FIRST ran @rostypegen in Main BEFORE loading Caesar
function PointCloud(pc2::Main.sensor_msgs.msg.PointCloud2)
  @error("work in progress on PointCloud(pc2::Main.sensor_msgs.msg.PointCloud2)")
  @show pc2.header.stamp
  header = Header(; # stamp = pc2.header.stamp,
                  seq    = pc2.header.seq,
                  frame_id= pc2.header.frame_id )
  PointCloud(;header,
              height     = pc2.height, 
              width      = pc2.width,
              point_step = pc2.point_step,
              row_step   = pc2.row_step )
              # is_dense   = pc2.is_dense )
end