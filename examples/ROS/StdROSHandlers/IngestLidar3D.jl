
"""
    $SIGNATURES

Message callback for Lidar. Adds a variable to the factor graph and appends the scan as a bigdata element.
"""
function handleLidarPointcloud!(
  msg::sensor_msgs.msg.PointCloud2, 
  dfg, 
  state::SystemState, 
  options::AbstractDict=Dict()
)
  @info "handleLidarPointcloud!" maxlog=3
  minRange = 1.5

  # convert the message to local memory type
  pc2 = Caesar._PCL.PCLPointCloud2(msg)
  sweep = Caesar._PCL.PointCloud(pc2)

  # add a new variable to the graph
  timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
  vartimestamp = string(unix2datetime(timestamp))*"Z"
  state.curtimestamp = timestamp
      
  lbl = "x$(state.var_index)"
  # pvLbl = state.prv_variable
  state.var_index += 1
  # t0 = 1.646305718997857e9
  # println("---LPC ts = $(timestamp-t0) cur = $(state.cur_var_timestamp-t0) $(state.cur_variable)")

  # check for valid data first
  # only add radar sweep variables every fifth pose
  if 0 < (state.var_index % state.stripeKeyframe) # FIXME, instead ` % options["poseEpochStripe"]`
    return nothing
  end
  # println("+++LPC writing to $lbl")
  state.cur_variable = lbl
  state.cur_var_timestamp = timestamp

  @info "Adding variable $lbl for PC"
  tags=["VARIABLE";"POSE"]
  # string(unix2datetime(timestamp)
  if state.pushNva # TODO is pushNva a global?
    addVariable!(
      dfg,
      lbl,
      :Pose3;
      tags,
      timestamp=vartimestamp
    )
  end
  
  # entries = Tuple{String,String,String}[]
  XYZfull = map(pt->pt.data[1:3], sweep.points )
  XYZfull_ = filter(pt->minRange < norm(pt), XYZfull)

  # TODO switch to Caesar's unmarshal into a Dict{String,Any}
  # header = unmarshal(msg.header)
  header = Dict{String, Any}( "seq" => msg.header.seq,
                              "stamp" => Dict{String,Int}("secs"=>msg.header.stamp.secs, "nsecs"=>msg.header.stamp.nsecs),
                              "frame_id" => msg.header.frame_id,
                              "_type" => "ROS1/std_msgs/Header" )

  if state.pushBlobs
    # store all the point cloud points in a blob store
    cloudDens = Vector{UInt8}(JSON.json(Dict("points"=>XYZfull_,"header"=>header)))
    cfid1 = addData(dfg, "pointcloud", cloudDens)
    mime = "application/json/octet-stream"
    eventId1 = addBlobEntry!( dfg, state.cur_variable, cfid1, 
                                "pointcloud", mime)
    # description="pc = JSON.parse(readBytes)"
    # push!(entries, ("pointcloud",cfid1,mime))
    # dataEntry_cloud1 = cfid1

    # store PCLPointCloud2 object as well (for headers and more)
    io = IOBuffer()
    # _PCL.saveLAS(io, _PCL.PointCloud(pc2)) # FIXME
    # Serialization.serialize(io,pc2)
    cfid1 = addData(dfg, "PCLPointCloud2", take!(io))
    mime = "application/serialize/octet-stream"
    eventId2 = addBlobEntry!( dfg, state.cur_variable, cfid1, 
                                "PCLPointCloud2", mime)
    #
  end

  state.prv_variable = lbl


  nothing
end
