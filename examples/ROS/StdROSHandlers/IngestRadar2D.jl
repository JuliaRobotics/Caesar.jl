

"""
    $SIGNATURES

Message callback for Radar pings. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleRadarPointcloud!(
  msg::sensor_msgs.msg.PointCloud2, 
  dfg, 
  state::SystemState, 
  options=Dict()
)
  @info "handleRadarPointcloud!" maxlog=3
  @info "options stripe" options["poseEpochStripe"]

  # latest epoch
  header = unmarshal(msg.header)
  timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
  state.curtimestamp = timestamp
  
  lbl = state.cur_variable
  
  # check if a new variable is needed
  # only add if this topic is responsible for new keyframe pose epochs (i.e. option greater than 0)
  if  (0 < options["poseEpochStripe"]) && 
      (0 == state.var_index % options["poseEpochStripe"]) && 
      state.pushNva
    # add a new variable to the graph
    lbl = "x$(state.var_index)"
    pvLbl = state.prv_variable
    state.prv_variable = state.cur_variable
    state.cur_variable = lbl
    state.var_index += 1 # FIXME, increment just before new variable, not after
    @info "Adding variable" lbl
    addVariable!(
      dfg,
      lbl,
      :Pose2; # Symbol(options["NVA_DIM_POSE"])
      tags=["VARIABLE";"POSE"],
      # timestamp = unix2datetime(timestamp),
    )  end
  
  # add radar blobs to latest variable (in JL format)
  pc2 = Caesar._PCL.PCLPointCloud2(msg)
  # sweep = Caesar._PCL.PointCloud(pc2)
  
  if state.pushBlobs
    # store PCLPointCloud2 object as well (for headers and more)
    bllb = RADARPOINTCLOUD(;suffix="_"*string(header["seq"]))
    # FIXME, include or offload the IncrSuffix for blob entries
    io = IOBuffer()
    Serialization.serialize(io,pc2)
    blobid = addData(dfg, bllb, take!(io))
    if state.pushNva
      mime = "application/serialize/octet-stream?_type=PCLPointCloud2"
      eventId2 = addBlobEntry!(dfg, lbl, blobid, 
                                  bllb, mime)
      #
    end
  end
  
  nothing
end



#