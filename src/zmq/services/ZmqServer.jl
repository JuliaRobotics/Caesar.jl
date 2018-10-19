using ZMQ
using Caesar, Caesar.ZmqCaesar

export
    start

import Base: start

systemverbs = Symbol[
    :shutdown
]
configverbs = Symbol[
    :initDfg;
    :toggleMockServer;
    :registerRobot;
    :registerSession;
]

sessionverbs = [
    :addVariable;
    :addFactor;
    :addOdometry2D;
    :addLandmark2D;
    :addFactorBearingRangeNormal;
    :ls;
    :getNode;
    :setReady;
    :batchSolve;
    # per variable
    :setVarKDE; # needed for workaround on bad autoinit -- sorry
    :getVarMAPKDE; # marginal belief points (KDE)
    :getVarMAPMean;
    # fancy future stuff
    :getVarMAPMax; # Future, how many maxes should you get?
    :getVarMAPFit; # default=Normal
]

plottingVerbs = [];
try
    # Test - if passed, declare verbs
    getfield(Main, :RoMEPlotting)
    plottingVerbs = [
        :plotKDE;
        :plotPose;
        :drawPoses;
        :drawPosesLandms
    ];
catch ex
    info("[ZMQ Server] Plotting is disabled!")
end

function shutdown(zmqServer, request)::Dict{String, Any}
    info("Shutting down ZMQ server on request...")
    zmqServer.isServerActive  = false
    return Dict{String, Any}("status" => "OK")
end

function start(zmqServer::ZmqServer)
    # set up a context for zmq
    Base.isopen(socket::Socket) = getfield(socket, :data) != C_NULL
    ctx=Context()
    s1=Socket(ctx, REP)
    ZMQ.bind(s1, "tcp://*:5555")

    try
        while zmqServer.isServerActive
            println("waiting to receive...")
            msg = ZMQ.recv(s1)
            out = ZMQ.convert(IOStream, msg)
            str = String(take!(out))
            request = JSON.parse(str)

            cmdtype = haskey(request, "type") ? Symbol(request["type"]) : :ERROR_NOCOMMANDPROVIDED
            info("[ZMQ Server] REQUEST: Received command '$cmdtype' in payload '$str'...")
            if cmdtype in union(configverbs, sessionverbs)
                resp = Dict{String, Any}()
                try
                    # Mocking server
                    if haskey(zmqServer.config, "isMockServer") && zmqServer.config["isMockServer"] == "true" && cmdtype != :toggleMockServer
                        warn("[ZMQ Server] MOCKING ENABLED - Ignoring request!")
                    else
                        # Otherwise actually perform the command
                        @show cmd = getfield(Caesar.ZmqCaesar, cmdtype)
                        resp = cmd(zmqServer.config, zmqServer.fg, request)
                    end
                    if !haskey(resp, "status")
                        resp["status"] = "OK"
                    end
                catch ex
                    io = IOBuffer()
                    showerror(io, ex, catch_backtrace())
                    err = String(take!(io))
                    warn("[ZMQ Server] Exception: $err")
                    resp["status"] = "ERROR"
                    resp["error"] = err
                end
                info("[ZMQ Server] RESPONSE: $(JSON.json(resp))")
                ZMQ.send(s1, JSON.json(resp))
            elseif cmdtype in systemverbs
                if cmdtype == :shutdown
                    #TODO: Call shutdown from here.
                    info("Shutting down ZMQ server on request...")
                    zmqServer.isServerActive  = false
                    resp = Dict{String, Any}("status" => "OK")
                    # Send response before shutting down.
                    ZMQ.send(s1, JSON.json(resp))
                end
            else
                warn("[ZMQ Server] Received invalid command $cmdtype")
                d = Dict{String, String}("status" => "ERROR", "error" => "Command '$cmdtype' not a valid command.")
                oks = json(d)
                ZMQ.send(s1, oks)
            end
        end
    catch ex
        warn("[ZMQ Server] Something in the zmq/json/rest pipeline broke")
        showerror(STDERR, ex, catch_backtrace())
    finally
        warn("[ZMQ Server] Shutting down server")
        # TODO: Figure out why I can't use Base.isopen here...
        if getfield(s1, :data) != C_NULL
            ZMQ.close(s1)
        end
        ZMQ.close(ctx)
        warn("[ZMQ Server] Shut down!")
    end
end
