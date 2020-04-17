
export RosbagSubscriber, loop!

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
  # constructors
  RosbagSubscriber() = new()
  RosbagSubscriber(bagfile::AbstractString;
                   channels::Vector{Symbol}=Symbol[],
                   callbacks::Dict{Symbol,Function}=Dict{Symbol,Function}(),
                   readers::Dict{Symbol,PyObject}=Dict{Symbol,PyObject}(),
                   syncBuffer::Dict{Symbol,Tuple{DateTime, Int}}=Dict{Symbol,Tuple{DateTime, Int}}() ) = new(bagfile,channels,callbacks,readers,syncBuffer)
end


function loop!(rbs::RosbagSubscriber)
  # figure out which channel is behind
  t1 = rbs.syncBuffer[rbs.channels[1]][1]
  times = (x->Nanosecond(rbs.syncBuffer[x][1]-t1)+Nanosecond(rbs.syncBuffer[x][2])).(rbs.channels)
  minT = minimum(times)
  idx = findfirst(x->x==minT, times)
  nextMsgChl = rbs.channels[idx]

  # get next event from that channel
  msg = rbs.readers[nextMsgChl].get_next_message()

  # try find event time
  msgT = msg[end]
  return if isa(msgT, PyObject)
    # update the syncBuffer
    rbs.syncBuffer[nextMsgChl] = (unix2datetime(msgT.to_sec()), msgT.to_nsec()%1000000000)
    # call the callback
    rbs.callbacks[nextMsgChl](msg)
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
