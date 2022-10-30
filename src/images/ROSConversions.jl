@info "Caesar.jl is loading tools relating to Images.jl and RobotOS.jl"

# not sure which of these are necessary with ros/CaesarROS.jl
using .RobotOS
import Unmarshal: unmarshal
# @rosimport sensor_msgs.msg: Image



function unmarshal(
  msg::Main.sensor_msgs.msg.Image
)
  Dict{String,Any}(
    "width"  => Int(msg.width),
    "height" => Int(msg.height),
    "step" => Int(msg.step),
    "data_b64" => msg.data, # base64encode(
    "encoding" => msg.encoding,
    "is_bigendian" => msg.is_bigendian === 0x01,
    "header" => unmarshal(msg.header),
    "_type" => "ROS1/sensor_msgs/Image?base64",
    "description" => "Caesar.toImage(JSON.parse(jstr))"
  )
end


toImage(msg::Main.sensor_msgs.msg.Image) = unmarshal(msg) |> toImage


"""
    $SIGNATURES

Convert `Caesar.Image::Dict` type to ROS message `sensor_msgs.msg.Image`.

See also: [`Caesar.unmarshal`](@ref), [`Caesar.toImage`](@ref), [`Caesar._PCL.toROSPointCloud2`](@ref)
"""
function toROSImage(msgd::Dict{String,Any})
  header = Main.std_msgs.msg.Header();
  header.seq = msgd["header"]["seq"]
  header.stamp = RobotOS.Time(msgd["header"]["stamp"]["secs"], msgd["header"]["stamp"]["nsecs"])
  header.frame_id = msgd["header"]["frame_id"]

  msg = Main.sensor_msgs.msg.Image();

  msg.header = header
  msg.height = UInt32(msgd["height"])
  msg.width  = UInt32(msgd["width"])

  msg.is_bigendian = UInt8(msgd["is_bigendian"])
  msg.step = UInt32(msgd["step"])
  msg.data = msgd["data_b64"] # base64decode( )
  msg.encoding = msgd["encoding"]

  msg
end
#