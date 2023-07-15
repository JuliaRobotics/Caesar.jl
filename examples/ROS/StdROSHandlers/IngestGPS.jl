

"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleGPS!(
  msg::sensor_msgs.msg.NavSatFix, 
  dfg, 
  state::SystemState, 
  options=Dict()
)
  @info "handleGPS" state.cur_variable state.prv_variable maxlog=10

  if state.cur_variable === nothing
    # skip if we don't have a variable yet.
    return nothing
  end

  if !haskey(state.workspace, "gps_keyframe")
    state.workspace["gps_keyframe"] = state.prv_variable
  end

  # blob label is user specified or Mapper default
  bllb = get(options, "name", GNSS_BLOB_LABEL())

  # quick and dirty synchronization of camera messages to lidar messages.
  # NOTE storing only the first image after a new pose is created
  if state.workspace["gps_keyframe"] == state.prv_variable
    return nothing
  end
  
  msgdict = unmarshal(msg)
  # @info "TRYING TO ADD" msg msgdict

  timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/10^9
  @info "[$timestamp] GPS sample on $(state.cur_variable)"
  
  bllb *= "_"*string(msg.header.seq) #msgdict["header"]["seq"])
  blob = JSON.json(msgdict) |> Vector{UInt8}
  mime = "application/json/octet-stream"

  if state.pushBlobs
    blobid = addData(dfg, bllb, blob)

    #
    addBlobEntry(
      dfg,
      # next of pair for now as rest works like that
      state.cur_variable,
      blobid,
      bllb,
      length(blob),
      mime
    )
  end

  state.workspace["gps_keyframe"] = state.prv_variable

  nothing
end


#