

function handleImu!(msg::sensor_msgs.msg.Imu, dfg, state::SystemState, options=Dict())
  @info "handleImu!" maxlog=10

  timestamp = Float64(msg.header.stamp.secs) + Float64(msg.header.stamp.nsecs)/1.0e9
  state.curtimestamp = timestamp


  if !haskey(state.workspace, "bufferImu")
    state.workspace["bufferImu"] = Dict{String,Any}("payload"=>[])
    state.workspace["bufferImu"]["basepose"] = state.cur_variable
    state.workspace["bufferImu"]["seq_start"] = msg.header.seq
  end

  di = unmarshal(msg)

  # add the current msg payload
  push!(state.workspace["bufferImu"]["payload"], di)

  # t0 = 1.646305718997857e9
  # println("---IMU ts = $(timestamp-t0) cur = $(state.cur_var_timestamp - t0)")

  # has a new pose keyframe epoch happened yet?
  # if state.workspace["bufferImu"]["basepose"] == state.cur_variable
  if timestamp <= state.cur_var_timestamp || state.workspace["bufferImu"]["basepose"] == state.cur_variable
    # still between the same poses, so dont push the blob up yet
    return nothing
  end

  state.workspace["bufferImu"]["seq_stop"] = msg.header.seq
  
  blob_lbl = get(options, "name", IMUDATABLOBNAME())
  blob_lbl *= "_"*string(state.workspace["bufferImu"]["seq_start"])
  blob_lbl *= "_"*string(state.workspace["bufferImu"]["seq_stop"]) 
  
  @info "adding imu to $(state.cur_variable)"
  # println("+++IMU write to $(state.cur_variable)")
  # after a new pose has been placed, then retroactively insert IMU data blob to previous pose

  if state.pushBlobs 
    io = IOBuffer()
    datastr = JSON.json(state.workspace["bufferImu"]["payload"])
    write(io, datastr)
    blob = take!(io)
    cfid1 = addData(dfg, blob_lbl, blob) |> fetch
    if state.pushNva
      mime = "application/json/octet-stream"
      addBlobEntry(
        dfg,
        # next of pair for now as rest works like that
        # state.workspace["bufferImu"]["basepose"],
        state.cur_variable,
        cfid1,
        blob_lbl,
        length(blob),
        mime
      )
    end
  end
  
  # after retroactive blob push, update to new base pose
  state.workspace["bufferImu"]["basepose"] = state.cur_variable
  state.workspace["bufferImu"]["seq_start"] = msg.header.seq
  state.workspace["bufferImu"]["seq_stop"] = -1
  state.workspace["bufferImu"]["payload"] = []

  return nothing
end