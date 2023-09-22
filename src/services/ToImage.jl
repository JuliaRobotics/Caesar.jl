

"""
    $SIGNATURES

Also see: [`PyCaesar.toROSImage`](@ref)
"""
function toImage(msgd::Dict{String,Any})
  data = haskey(msgd, "data_b64") ? base64decode(msgd["data_b64"]) : UInt8.(msgd["data"])
  h, w = msgd["height"], msgd["width"]
  @show h, w, msgd["step"]
  if msgd["encoding"] == "mono8" || msgd["encoding"] == "8UC1"
    # img_ = normedview(N0f8, data)
    # reshape(img_, w, h)'
    img = Matrix{Gray{N0f8}}(undef, h, w)
    # assuming one endian type for now, TODO both little and big endian
    for i in 1:h, j in 1:w
      img[i,j] = Gray{N0f8}(data[msgd["step"]*(i-1)+j]/(2^8 - 1))
    end
    img
  elseif msgd["encoding"] == "mono16" || msgd["encoding"] == "16UC1"
    img_ = normedview(N0f16, data)
    reshape(img_, w, h)'
  else
    error("Conversion for ROS sensor_msgs.Image encoding not implemented yet $(msgd["encoding"])")
  end
end