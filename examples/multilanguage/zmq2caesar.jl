# receive messages on ZMQ

using ZMQ, JSON
using Caesar
using Unmarshal


# getfield(Caesar, :registerRobot)

# 1a. Create a Configuration
# synchronyConfig = loadConfig("synchronyConfig_Local.json")
robotId = ""
sessionId = ""
# synchronyConfig = loadConfig(joinpath(ENV["HOME"],"Documents","synchronyConfig.json"))

# 1b. Check the credentials and the service status
# @show serviceStatus = getStatus(synchronyConfig)

# set up a context for zmq
ctx=Context()
s1=Socket(ctx, REP)

ZMQ.bind(s1, "tcp://*:5555")

configverbs = Symbol[
  :registerRobot;
  :registerSession;
]

sessionverbs = [
  :addOdometry2D;
  :addLandmark2D;
  :addFactorBearingRangeNormal;
  :setReady;
  :addVariable;
  :addFactor
]


0

fg = Caesar.initfg()

config = Dict{String, String}()

try
  while true
    println("waiting to receive...")
    msg = ZMQ.recv(s1)
    out=convert(IOStream, msg)

    @show str = String(take!(out))
    @show request = JSON.parse(str)

    @show cmdtype = Symbol(request["type"])
    if cmdtype in union(configverbs, sessionverbs)
      @show cmd = getfield(Caesar, cmdtype)

      if cmdtype in configverbs
        resp = cmd(config, request)
      elseif cmdtype in sessionverbs
        resp = cmd(config, fg, request)
      end

      d = Dict{String, String}()
      d["status"] = length(resp) == 0 ? "OK" : resp
      oks = json(d)
      ZMQ.send(s1, oks)
    else
      warn("received invalid command $cmdtype")
      d = Dict{String, String}("status" => "KO")
      oks = json(d)
      ZMQ.send(s1, oks)
    end
  end
catch ex
  warn("Something in the zmq/json/rest pipeline broke")
  showerror(STDERR, ex, catch_backtrace())
finally
  ZMQ.close(s1)
  # ZMQ.close(s2)
  ZMQ.close(ctx)
end

0


# registerRobot
# registerSession
# addOdometry2D # specialized convenience
# addLandmark2D # specialized convenience
# addFactorBearingRangeNormal # specialized convenience
# setReady # be wary, require full atomic transactions across graph segments
# addVariable
# addFactor
# getBelief
# getBeliefMax
# triggerSolve





#
