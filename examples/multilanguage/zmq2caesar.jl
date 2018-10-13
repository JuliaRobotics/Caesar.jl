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

systemverbs = Symbol[
    :shutdown
]
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
global isServerActive = true
function shutdown(config, fg, request)::Dict{String, Any}
    info("Shutting down ZMQ server on request...")
    global isServerActive
    isServerActive  = false
    return Dict{String, Any}("status" => "OK")
end

# set up a context for zmq
Base.isopen(socket::Socket) = getfield(socket, :data) != C_NULL
ctx=Context()
s1=Socket(ctx, REP)
ZMQ.bind(s1, "tcp://*:5555")

try
    while isServerActive
        println("waiting to receive...")
        msg = ZMQ.recv(s1)
        out=convert(IOStream, msg)
        str = String(take!(out))
        request = JSON.parse(str)

        @show cmdtype = haskey(request, "type") ? Symbol(request["type"]) : :NOTYPEKEY
        info("Received command '$cmdtype' in payload '$str'...")
        if cmdtype in union(configverbs, sessionverbs)
            @show cmd = getfield(Caesar, cmdtype)

            resp = Dict{String, Any}()
            try
                resp = cmd(config, fg, request)
                if !haskey(resp, "status")
                    resp["status"] = "OK"
                end
            catch ex
                io = IOBuffer()
                showerror(io, e, catch_backtrace())
                err = String(take!(io))
                warn("[ZMQ Server] Exception: $err")
                resp["status"] = "ERROR"
                resp["error"] = err
            end
            ZMQ.send(s1, JSON.json(resp))
        elseif cmdtype in systemverbs
            if cmdtype == :shutdown
                shutdown(config, fg, request)
            end
        else
            warn("received invalid command $cmdtype")
            d = Dict{String, String}("status" => "ERROR")
            oks = json(d)
            ZMQ.send(s1, oks)
        end
    end
catch ex
    warn("Something in the zmq/json/rest pipeline broke")
    showerror(STDERR, ex, catch_backtrace())
finally
    warn("[ZMQ Server] Shutting down server")
    if isopen(s1)
        ZMQ.close(s1)
    end
    ZMQ.close(ctx)
    warn("[ZMQ Server] Shut down!")
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
