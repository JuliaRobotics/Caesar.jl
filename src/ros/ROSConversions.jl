

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


