
export RosbagSubscriber, loop!, getROSPyMsgTimestamp, nanosecond2datetime

## Load rosbag file parser

pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__ )

rosbagReaderPy = pyimport("rosbagReader")
RosbagParser = rosbagReaderPy."RosbagParser"


## Common handler approach

mutable struct RosbagSubscriber
  bagfile::String
  channels::Vector{Symbol}
  callbacks::Dict{Symbol, Function}
  readers::Dict{Symbol,PyObject}
  syncBuffer::Dict{Symbol,Tuple{DateTime, Int}} # t,ns,msgdata
  nextMsgChl::Symbol
  nextMsgTimestamp::Tuple{DateTime, Int}
  # constructors
end
RosbagSubscriber(bagfile::AbstractString;
                 channels::Vector{Symbol}=Symbol[],
                 callbacks::Dict{Symbol,Function}=Dict{Symbol,Function}(),
                 readers::Dict{Symbol,PyObject}=Dict{Symbol,PyObject}(),
                 syncBuffer::Dict{Symbol,Tuple{DateTime, Int}}=Dict{Symbol,Tuple{DateTime, Int}}() ) = RosbagSubscriber(bagfile,channels,callbacks,readers,syncBuffer, :null, (unix2datetime(0),0))
#

# loss of accuracy (Julia only Millisecond)
function nanosecond2zoneddatetime(nsT::Int64)
  nano = nsT % 1_000_000_000
  secs = nsT - (nano % 1_000_000)
  secs /= 1_000_000_000
  ZonedDateTime(unix2datetime(secs), localzone())
end
@deprecate nanosecond2datetime(x...) nanosecond2zoneddatetime(x...)

function getROSPyMsgTimestamp(msgT::PyObject)
  msgNs = msgT.to_sec()
  # jlSeconds = isa(msg, PyObject) ? msgNs : msgNs
  jlSeconds = floor(Int64, Float64(msgNs))
  jlNanosec = msgT.to_nsec()%1000000000
  # try use regular seconds if no Nanosecond information is available
  jlNanosec = floor(Int,jlNanosec) == 0 && 0 < Float64(msgNs) % 1.0 |> abs ? Float64(msgNs) % 1.0 : jlNanosec
  (unix2datetime(jlSeconds), jlNanosec)
end

function loop!(rbs::RosbagSubscriber, args...)
  # figure out which channel is behind
  t1 = rbs.syncBuffer[rbs.channels[1]][1]
  times = (x->Nanosecond(rbs.syncBuffer[x][1]-t1)+Nanosecond(rbs.syncBuffer[x][2])).(rbs.channels)
  minT = minimum(times)
  idx = findfirst(x->x==minT, times)
  rbs.nextMsgChl = rbs.channels[idx]

  # get next event from that channel
  msg = rbs.readers[rbs.nextMsgChl].get_next_message()

  # try find event time
  msgT = msg[end]
  return if isa(msgT, PyObject)
    # update the syncBuffer
    rbs.syncBuffer[rbs.nextMsgChl] = rbs.nextMsgTimestamp = getROSPyMsgTimestamp(msgT)
    # call the callback
    rbs.callbacks[rbs.nextMsgChl](msg, args...)
    true
  else
    println("false, dont know how to handle message time as typeof(msgT)=$(typeof(msgT))")
    @show msg
    false
  end
end

function (rbs::RosbagSubscriber)(chl::AbstractString,
                                 callback::Function,
                                 args...;
                                 msgType=nothing)
  #
  cn = Symbol(string(chl))
  push!(rbs.channels, cn)
  rbs.callbacks[cn] = (m)->callback(m,args...)
  rbs.syncBuffer[cn] = (unix2datetime(0), 0)
  rbs.readers[cn] = RosbagParser(rbs.bagfile, chl)
end
