

function handleLaserScan!(msg::sensor_msgs.msg.LaserScan, dfg, state::SystemState, options=Dict())
  @info "handleLaserScan! (aka 2D Lidar)" maxlog=10

  state.var_index += 1

  # @show msg.header
  if Int(msg.header.seq) % state.stripeKeyframe !== 0
    return nothing
  end

  lp_ = lg.LaserProjection()

  # add a new variable to the graph
  timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
  state.curtimestamp = timestamp
      
  lbl = "x$(state.var_index)"
  # pvLbl = state.prv_variable
  state.cur_variable = lbl

  # add a new pose
  resultIds = Task[]
  @info "Adding variable" lbl
  tags=["VARIABLE";"POSE"]
  
  if state.pushNva
    # string(unix2datetime(timestamp)
    addVariable(
      dfg,
      lbl,
      :Pose2;
      # tags
    )
  end

  _pc2_msg = lp_.projectLaser(msg) #PyObject
  pc2_msg = convert(sensor_msgs.msg.PointCloud2, _pc2_msg); # Julia object

  # convert the message to local memory type
  pc2 = Caesar._PCL.PCLPointCloud2(pc2_msg)
  # sweep = Caesar._PCL.PointCloud(pc2)


  # store PCLPointCloud2 object as well (for headers and more)
  if state.pushBlobs 
    blob_lbl = "PCLPointCloud2_"*string(Int(msg.header.seq))
    io = IOBuffer()
    # FIXME, update to pcd format (not just Julia.Serialization.serialize): https://github.com/JuliaRobotics/Caesar.jl/issues/921
    Serialization.serialize(io,pc2)
    cfid1 = addData(dfg, blob_lbl, take!(io))
    if state.pushNva
      mime = "application/serialize/octet-stream"
      addBlobEntry!( dfg, state.cur_variable, cfid1, 
                        blob_lbl, mime)
    end
  end

  state.prv_variable = lbl

  nothing
end

@deprecate handleLidar!(w...; kw...) handleLaserScan!(w...; kw...)