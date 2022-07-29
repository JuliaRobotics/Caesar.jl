
@info "Loading Caesar ROS specific utilities (using RobotOS)."


export RosbagSubscriber, loop!, getROSPyMsgTimestamp, nanosecond2datetime
export RosbagWriter

## Load rosbag file parser

pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__ )

rosbagReaderPy = pyimport("rosbagReader")
RosbagParser = rosbagReaderPy."RosbagParser"

# piggy back writer code here
writeRosbagPy = pyimport("rosbagWriter")

"""
    RosbagWriter(bagfilepath)

```julia
# Link with ROSbag infrastructure via rospy
using Pkg
ENV["PYTHON"] = "/usr/bin/python3"
Pkg.build("PyCall")
using PyCall
using RobotOS
@rosimport std_msgs.msg: String
rostypegen()
using Caesar

bagwr = Caesar.RosbagWriter("/tmp/test.bag")
s = std_msgs.msg.StringMsg("test")
bagwr.write_message("/ch1", s)
bagwr.close()
```
"""
struct RosbagWriter end
(::Type{RosbagWriter})(args...; kwargs...) = writeRosbagPy."RosbagWriter"(args...; kwargs...)

## Common handler approach

mutable struct RosbagSubscriber
  bagfile::String
  channels::Vector{Symbol}
  callbacks::Dict{Symbol, Function}
  readers::Dict{Symbol,PyObject}
  syncBuffer::Dict{Symbol,Tuple{DateTime, Int}} # t,ns,msgdata
  nextMsgChl::Symbol
  nextMsgTimestamp::Tuple{DateTime, Int}
  compression::Any
  # constructors
end
RosbagSubscriber(bagfile::AbstractString;
                  compression=nothing,
                  channels::Vector{Symbol}=Symbol[],
                  callbacks::Dict{Symbol,Function}=Dict{Symbol,Function}(),
                  readers::Dict{Symbol,PyObject}=Dict{Symbol,PyObject}(),
                  syncBuffer::Dict{Symbol,Tuple{DateTime, Int}}=Dict{Symbol,Tuple{DateTime, Int}}() ) = RosbagSubscriber(bagfile,channels,callbacks,readers,syncBuffer, :null, (unix2datetime(0),0),compression)
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
    @debug "RosbagSubscriber got msg $(rbs.nextMsgChl)"
    # call the callback
    rbs.callbacks[rbs.nextMsgChl](msg, args...)
    true
  else
    @warn "false, dont know how to handle message" rbs.nextMsgChl msgT maxlog=10
    # @show msg
    false
  end
end

function (rbs::RosbagSubscriber)( chl::AbstractString,
                                  ::Type{MT},
                                  callback::Function,
                                  args::Tuple;
                                  msgType=nothing ) where {MT <: RobotOS.AbstractMsg} 
  #
  cn = Symbol(string(chl))
  push!(rbs.channels, cn)
  # include the type converter, see ref: https://github.com/jdlangs/RobotOS.jl/blob/21a7088461a21bc9b24cd2763254d5043d5b1800/src/callbacks.jl#L23
  rbs.callbacks[cn] = (m)->callback(convert(MT,m[2]),args...)
  rbs.syncBuffer[cn] = (unix2datetime(0), 0)
  rbs.readers[cn] = RosbagParser(rbs.bagfile, chl) #; rbs.compression)
end
