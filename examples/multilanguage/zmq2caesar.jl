# receive messages on ZMQ

using Caesar
using ZMQ, JSON


# 1. Import the initialization code.
include(joinpath(Pkg.dir("SynchronySDK"),"examples", "0_Initialization.jl"))

# 1a. Create a Configuration
# synchronyConfig = loadConfig("synchronyConfig_Local.json")
robotId = ""  # bad Sam
sessionId = ""
synchronyConfig = loadConfig(joinpath(ENV["HOME"],"Documents","synchronyConfig.json"))

# 1b. Check the credentials and the service status
@show serviceStatus = getStatus(synchronyConfig)

# set up a context for zmq
ctx=Context()
s1=Socket(ctx, REP)

ZMQ.bind(s1, "tcp://*:5555")

try
  while true
    msg = ZMQ.recv(s1)
    out=convert(IOStream, msg)

    str = takebuf_string(out)

    dict = JSON.parse(str)

    robotId = dict["robotId"]  # bad Sam
    sessionId = dict["sessionId"]

    @show dict["type"]
    @show cmd = getfield(SynchronySDK, Symbol(dict["type"]))

    args = (synchronyConfig,)
    @show cmd(args...)
  end
catch ex
  warn("Something in the zmq/json/rest pipeline broke")
  showerror(STDERR, ex, catch_backtrace())
finally
  ZMQ.close(s1)
  # ZMQ.close(s2)
  ZMQ.close(ctx)
end




#
