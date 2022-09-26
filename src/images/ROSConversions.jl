@info "Caesar.jl is loading tools relating to Images.jl and RobotOS.jl"

using .RobotOS

import Unmarshal: unmarshal

@rosimport std_msgs.msg: Header
@rosimport sensor_msgs.msg: Image


function unmarshal(
  header::Main.std_msgs.msg.Header
)
  #
  Dict{String,Any}( 
    "seq" => header.seq,
    "stamp" => Dict{String,Int}(
      "secs"=>header.stamp.secs, 
      "nsecs"=>header.stamp.nsecs
    ),
    "frame_id" => header.frame_id,
    "_type" => "ROS1/std_msgs/Header" 
  )
end

function unmarshal(
  msg::Main.sensor_msgs.msg.Image
)
  Dict{String,Any}(
    "width"  => Int(msg.width),
    "height" => Int(msg.height),
    "step" => Int(msg.step),
    "data_b64" => base64encode(msg.data),
    "encoding" => msg.encoding,
    "is_bigendian" => msg.is_bigendian === 0x01,
    "header" => unmarshal(msg.header),
    "_type" => "ROS1/sensor_msgs/Image?base64",
    "description" => "Caesar.toImage(JSON.parse(jstr))"
  )
end


toImage(msg::Main.sensor_msgs.msg.Image) = unmarshal(msg) |> toImage

#