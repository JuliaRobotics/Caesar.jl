@info "Caesar.jl is loading tools relating to Images.jl and RobotOS.jl"

using .RobotOS

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
    "_type" => "ROS1/std_msgs/Image?base64"
  )
end




function toImage(msg::Main.sensor_msgs.msg.Image)
  # header = Header(;stamp  = msg.header.stamp,
  #                 seq     = msg.header.seq,
  #                 frame_id= msg.header.frame_id )
  #
  w, h = Int(msg.width), Int(msg.height)

  @info "sensor_msgs.msg.Image" msg.encoding w h Int(msg.step)

  if msg.encoding == "mono8"
    img = Matrix{Gray{N0f8}}(undef, h, w)
    # assuming one endian type for now, TODO both little and big endian
    for i in 1:h, j in 1:w
      img[i,j] = Gray{N0f8}(msg.data[msg.step*(i-1)+j]/255)
    end
    img
  else
    error("Conversion for ROS sensor_msgs.Image encoding not implemented yet $(msg.encoding)")
  end
end