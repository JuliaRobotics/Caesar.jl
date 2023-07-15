
"""
$SIGNATURES

Message callback for camera images.
"""
function handleCameraRaw!(
  msg::sensor_msgs.msg.Image, 
  dfg, 
  state::SystemState, 
  options=Dict()
)
  # must this handler addVariables as poses on each epoch?
  doEpochPoses = haskey(options, "poseEpochStripe")
  # specific parameters
  # lbl = "IMG_CENTER_$(state.cur_variable)_$(msg.header.seq)"
  # user specified blob name, or use default
  # blob_lbl = CAMERA_BLOBNAME()
  blob_lbl = get(options, "name", CAMERA_BLOBNAME()) # "cam0"
  @info "ingestCameraRaw!" blob_lbl maxlog=20

  # img = Caesar.toImage(msg)
  # @info "Image data" msg.encoding Int(msg.width) Int(msg.height) Int(msg.step)

  if !haskey(state.workspace, blob_lbl)
    state.workspace[blob_lbl] = state.prv_variable
  end

  # @info "THIS FAR" blob_lbl state.workspace[blob_lbl] state.prv_variable

  # prvLbl = state.prv_variable === "" ? "x0" : state.prv_variable
  # quick and dirty synchronization of camera messages to lidar messages.
  # NOTE storing only the first image after a new pose is created
  if  (!doEpochPoses) &&
      (state.workspace[blob_lbl] == state.prv_variable)
    @info "SKIP" blob_lbl state.workspace[blob_lbl]
    return nothing
  end
  
  # @info "AND MORE" blob_lbl state.prv_variable

  # if this handler should be used to select key frames and make poses
  if doEpochPoses
    timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
    vartimestamp = string(unix2datetime(timestamp))*"Z"
    state.curtimestamp = timestamp
    lbl = "x$(state.var_index)"
    state.var_index += 1
      # check for valid data first
    # only add radar sweep variables every fifth pose
    if 0 < (state.var_index % state.stripeKeyframe) # TBD, ` % options["poseEpochStripe"]` vs Mapper global pose stripe count?
      return nothing
    end
    # println("+++LPC writing to $lbl")
    state.cur_variable = lbl
    state.cur_var_timestamp = timestamp

    @info "Adding variable $lbl for Image"
    tags=["VARIABLE";"POSE"]
    # string(unix2datetime(timestamp)
    if state.pushNva
      addVariable!(
          dfg,
          lbl,
          :Pose3; # TODO recover from user options if available
          tags,
          timestamp=vartimestamp
      )
    end
  end
  
  
  imgd = Caesar.unmarshal(msg)
  # img = Caesar.toImage(msg)

  blob_lbl *= "_"*string(imgd["header"]["seq"])
  blob = JSON.json(imgd) |> Vector{UInt8}
  mime = "application/json/"*imgd["_type"] # ROS1/sensor_msgs/Image?base64"

  if state.pushBlobs
    @info "ADD BLOB" state.cur_variable blob_lbl
    blobid = addData(dfg, blob_lbl, blob)
    
    # not super well synched, but good enough for basic demonstration
      # description="Caesar.toImage(JSON.parse(blobstr))"
    addBlobEntry!(
      dfg,
      state.cur_variable,
      blobid,
      blob_lbl,
      length(blob),
      mime,
    )
  end

  if doEpochPoses
    state.prv_variable = lbl
  end

  # TODO write reason for why we use prv_variable here in state.workspace register?
  state.workspace[blob_lbl] = state.prv_variable


  nothing
end
