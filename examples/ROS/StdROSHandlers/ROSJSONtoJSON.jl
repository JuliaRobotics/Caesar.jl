
# did not debug errors when trying to move these unmarshal functions upstream
# standard types
# @rosimport sensor_msgs.msg: CompressedImage
@rosimport tf2_msgs.msg: TFMessage

@rosimport nav_msgs.msg: Odometry

@rosimport sensor_msgs.msg: PointCloud2
@rosimport sensor_msgs.msg: LaserScan
@rosimport sensor_msgs.msg: Image
@rosimport sensor_msgs.msg: Imu
@rosimport sensor_msgs.msg: NavSatFix
@rosimport sensor_msgs.msg: RegionOfInterest
@rosimport sensor_msgs.msg: CameraInfo

@rosimport geometry_msgs.msg: TransformStamped
@rosimport geometry_msgs.msg: Transform
@rosimport geometry_msgs.msg: Vector3
@rosimport geometry_msgs.msg: Quaternion
@rosimport geometry_msgs.msg: Pose
@rosimport geometry_msgs.msg: Point
@rosimport geometry_msgs.msg: Vector3
@rosimport geometry_msgs.msg: Twist
@rosimport geometry_msgs.msg: TwistWithCovariance
@rosimport geometry_msgs.msg: TwistWithCovarianceStamped

# do not load Caesar until after rostypegen
rostypegen()

# dont load Caesar until after rostypegen
import Caesar: unmarshal

##

# WORKAROUND for getting string input for for PyCall defined types in Julia Main
# usual approach did not work, getfield(Main, Symbol("sensor_msgs.msg.LaserScan"))
ROS_MSG_LOOKUP_DICT = Dict(
  "tf2_msgs.msg.TFMessage" => tf2_msgs.msg.TFMessage,
  #
  "nav_msgs.msg.Odometry" => nav_msgs.msg.Odometry,
  #
  "sensor_msgs.msg.PointCloud2" => sensor_msgs.msg.PointCloud2, # in Caesar._PCL
  "sensor_msgs.msg.LaserScan" => sensor_msgs.msg.LaserScan, # TODO build converter
  "sensor_msgs.msg.Image" => sensor_msgs.msg.Image, # in Caesar
  "sensor_msgs.msg.Imu" => sensor_msgs.msg.Imu,
  "sensor_msgs.msg.NavSatFix" => sensor_msgs.msg.NavSatFix,
  "sensor_msgs.msg.RegionOfInterest" => sensor_msgs.msg.RegionOfInterest,
  "sensor_msgs.msg.CameraInfo" => sensor_msgs.msg.CameraInfo,
  #
  "geometry_msgs.msg.Vector3" => geometry_msgs.msg.Vector3,
  "geometry_msgs.msg.Point" => geometry_msgs.msg.Point,
  "geometry_msgs.msg.Quaternion" => geometry_msgs.msg.Quaternion,
  "geometry_msgs.msg.Pose" => geometry_msgs.msg.Pose,
  "geometry_msgs.msg.Transform" => geometry_msgs.msg.Transform,
  "geometry_msgs.msg.TransformStamped" => geometry_msgs.msg.TransformStamped,
  "geometry_msgs.msg.Twist" => geometry_msgs.msg.Twist,
  "geometry_msgs.msg.TwistWithCovariance" => geometry_msgs.msg.TwistWithCovariance,
  "geometry_msgs.msg.TwistWithCovarianceStamped" => geometry_msgs.msg.TwistWithCovarianceStamped,
  "geometry_msgs.msg.PoseWithCovariance" => geometry_msgs.msg.PoseWithCovariance,
  #
)

##

function unmarshal(msg::Main.tf2_msgs.msg.TFMessage)
  Dict{String,Any}(
    "transforms" => unmarshal.(msg.transforms),
    "_type" => "ROS1/tf2_msgs/TFMessage"
  )
end

function unmarshal(msg::Main.nav_msgs.msg.Odometry)
  # http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  Dict{String,Any}(
    "header" => Caesar.unmarshal(msg.header),
    "child_frame_id" => msg.child_frame_id,
    "pose" => unmarshal(msg.pose),
    "twist" => unmarshal(msg.twist),
    "_type" => "ROS1/nav_msgs/Odometry",
  )
end

## ==================

function unmarshal(msg::Main.sensor_msgs.msg.RegionOfInterest)
  Dict{String,Any}(
    "x_offset" => msg.x_offset,
    "y_offset" => msg.y_offset,
    "height" => msg.height,
    "width" => msg.width,
    "do_rectify" => msg.do_rectify,
    "_type" => "ROS1/sensor_msgs/RegionOfInterest",
  )
end


function unmarshal(msg::Main.sensor_msgs.msg.CameraInfo)
  # https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/CameraInfo.html
  Dict{String,Any}(
    "header" => unmarshal(msg.header),
    "height" => msg.height,
    "width" => msg.width,
    "distortion_model" => msg.distortion_model,
    "D" => msg.D,
    "K" => msg.K,
    "R" => msg.R,
    "P" => msg.P,
    "binning_x" => msg.binning_x,
    "binning_y" => msg.binning_y,
    "roi" => unmarshal(msg.roi),
    "_type" => "ROS1/sensor_msgs/CameraInfo",
  )
end

function unmarshal(msg::Main.sensor_msgs.msg.Imu)
  Dict{String,Any}(
    "header" => unmarshal(msg.header),
    "orientation" => msg.orientation,
    "orientation_covariance" => msg.orientation_covariance,

    "angular_velocity" => msg.angular_velocity,
    "angular_velocity_covariance" => msg.angular_velocity_covariance,

    "linear_acceleration" => msg.linear_acceleration,
    "linear_acceleration_covariance" => msg.linear_acceleration_covariance,
    "_type" => "ROS1/sensor_msgs/Imu",
  )
end

function unmarshal(msg::Main.sensor_msgs.msg.NavSatFix)
  # http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  Dict{String,Any}(
    "header" => unmarshal(msg.header),
    "status" => Dict{String, UInt16}(
      "status" => msg.status.status,
      "service" => msg.status.service,
    ),
    "latitude" => msg.latitude,
    "longitude" => msg.longitude,
    "altitude" => msg.altitude,
    "position_covariance" => msg.position_covariance,
    "position_covariance_type" => msg.position_covariance_type,
    "_type" => "ROS1/sensor_msgs/NavSatFix",
  )
end



## =============


function unmarshal(msg::Main.geometry_msgs.msg.Quaternion)
  Dict{String,Any}(
    "x" => msg.x,
    "y" => msg.y,
    "z" => msg.z,
    "w" => msg.w,
    "_type" => "ROS1/geometry_msgs/Quaternion"
  )
end
function unmarshal(msg::Main.geometry_msgs.msg.Transform)
  Dict{String,Any}(
    "translation" => unmarshal(msg.translation),
    "rotation" => unmarshal(msg.rotation),
    "_type" => "ROS1/geometry_msgs/Transform",
  )
end
function unmarshal(msg::Main.geometry_msgs.msg.TransformStamped)
  Dict{String,Any}(
    "header" => Caesar.unmarshal(msg.header),
    "child_frame_id" => msg.child_frame_id,
    "transform" => unmarshal(msg.transform),
    "_type" => "ROS1/geometry_msgs/TransformStamped"
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.Vector3)
  Dict{String,Any}(
    "x" => msg.x,
    "y" => msg.y,
    "z" => msg.z,
    "_type" => "ROS1/geometry_msgs/Vector3"
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.Point)
  # http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html
  Dict{String,Any}(
    "x" => msg.x,
    "y" => msg.y,
    "z" => msg.z,
    "_type" => "ROS1/geometry_msgs/Point",
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.Pose)
  Dict{String,Any}(
    "position" => unmarshal(msg.position),
    "orientation" => unmarshal(msg.orientation),
    "_type" => "ROS1/geometry_msgs/Pose",
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.Twist)
  # http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
  Dict{String,Any}(
    "linear" => unmarshal(msg.linear),
    "angular" => unmarshal(msg.angular),
    "_type" => "ROS1/geometry_msgs/Twist",
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.TwistWithCovariance)
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovariance.html
    Dict{String,Any}(
      "twist" => unmarshal(msg.twist),
      "covariance" => float.(msg.covariance),
      "_type" => "ROS1/geometry_msgs/TwistWithCovariance"
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.TwistWithCovarianceStamped)
  Dict{String,Any}(
    "header" => Caesar.unmarshal(msg.header),
    "twist" => unmarshal(msg.twist),
    "_type" => "ROS1/geometry_msgs/TwistWithCovarianceStamped"
  )
end

function unmarshal(msg::Main.geometry_msgs.msg.PoseWithCovariance)
  # http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Dict{String,Any}(
    "pose" => unmarshal(msg.pose),
    "covariance" => float.(msg.covariance),
    "_type" => "ROS1/geometry_msgs/PoseWithCovariance",
  )
end
