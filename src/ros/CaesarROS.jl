# tools to load ROS environment

@info "Loading Caesar PyCall specific utilities (using PyCall)."

using .PyCall
using .RobotOS

import Unmarshal: unmarshal

@rosimport std_msgs.msg: Header

# standard types
@rosimport sensor_msgs.msg: PointCloud2
@rosimport sensor_msgs.msg: LaserScan
@rosimport sensor_msgs.msg: CompressedImage
@rosimport sensor_msgs.msg: Image
@rosimport sensor_msgs.msg: Imu
@rosimport tf2_msgs.msg: TFMessage
@rosimport geometry_msgs.msg: TransformStamped
@rosimport geometry_msgs.msg: Transform
@rosimport geometry_msgs.msg: Vector3
@rosimport geometry_msgs.msg: Quaternion
@rosimport nav_msgs.msg: Odometry

# rostypegen()

include("Utils/RosbagSubscriber.jl")
include("ROSConversions.jl")
