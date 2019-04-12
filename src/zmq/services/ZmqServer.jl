using ZMQ
using Caesar, Caesar.ZmqCaesar
using Dates

export
    start,
    processJsonDir

# import Base: start

global systemverbs = Symbol[
    :shutdown;
    :resetfg;
    :savecheckpoint
]
global configverbs = Symbol[
    :initDfg;
    :toggleMockServer;
    :registerRobot;
    :registerSession;
]

global sessionverbs = [
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

global additionalVerbs = [
    :multiplyDistributions;
    :status
];

global plottingVerbs = [];
try
    # Test - if passed, declare verbs
    getfield(Main, :RoMEPlotting)
    global plottingVerbs = [
        :plotKDE;
        :plotPose;
        :drawPoses;
        :drawPosesLandms
    ];
catch ex
    @debug "[ZMQ Server] Plotting is disabled!"
end

function shutdown(zmqServer, request)::Dict{String, Any}
    @info "Shutting down ZMQ server on request..."
    zmqServer.isServerActive  = false
    return Dict{String, Any}("status" => "OK")
end

function Base.isopen(socket::Socket)
    return getfield(socket, :data) != C_NULL
end

function logEventFile(logdir, msg; eventtime=round(Int,time()*1e6))::Nothing
  msgl = deepcopy(msg)
  @async begin
    fid = open(joinpath(logdir,"$eventtime.json"), "w")
    write(fid, msgl)
    close(fid)
  end
  nothing
end

"""
    $(SIGNATURES)

Process JSON strings
"""
function processJsonString(zmqServer::ZmqServer, str::String, drawcounter::Int)
  global systemverbs
  global configverbs
  global sessionverbs
  global additionalVerbs
  global plottingVerbs

  request = JSON.parse(str)
  resp = Dict{String, Any}()

  cmdtype = haskey(request, "request") ? Symbol(request["request"]) : :ERROR_NOCOMMANDPROVIDED
  @info "[ZMQ Server] REQUEST: Received request '$cmdtype' in payload '$str'..."
  if cmdtype in union(configverbs, sessionverbs, additionalVerbs)
      try
          # Mocking server
          if haskey(zmqServer.config, "isMockServer") && zmqServer.config["isMockServer"] == "true" && cmdtype != :toggleMockServer
              @warn "[ZMQ Server] MOCKING ENABLED - Ignoring request!"
          else
              # Otherwise actually perform the command
              @show cmd = getfield(Caesar.ZmqCaesar, cmdtype)
              resp = cmd(zmqServer.config, zmqServer.fg, request)
          end
          if !haskey(resp, "status")
              resp["status"] = "OK"
              drawcounter += 1
          end
          # TODO: make this an ArgParse.jl enableable feature
          if drawcounter % 20 == 0
              IIF.writeGraphPdf(zmqServer.fg, show=true)
          else
              drawcounter = 0
          end
      catch ex
          io = IOBuffer()
          showerror(io, ex, catch_backtrace())
          err = String(take!(io))
          @warn "[ZMQ Server] Exception: $err"
          resp["status"] = "ERROR"
          resp["error"] = err
      end
      @info "[ZMQ Server] RESPONSE: $(JSON.json(resp))"
  elseif cmdtype in systemverbs
      if cmdtype == :shutdown
          #TODO: Call shutdown from here.
          @info "Shutting down ZMQ server on request..."
          zmqServer.isServerActive  = false
          resp = Dict{String, Any}("status" => "OK")
          # Send response before shutting down.
      elseif cmdtype == :resetfg
          @info "Resetting the factor graph..."
          zmqServer.fg = initfg()
          resp = Dict{String,Any}("status" => "OK")
          # Send response before shutting down.
      elseif cmdtype == :savecheckpoint
          timest = Dates.format(now(), DateFormat("yyyy-mm-dd_HHMMSS"))
          filename = "/tmp/zmqFG-$(timest).jld2"
          @info "saving checkpoint to $filename"
          savejld(zmqServer.fg, file=filename)
          resp = Dict{String,Any}("status" => "OK", "destination" => filename)
          # Send response before shutting down.
      else
          error("zmqServer: unknown systemverbs command $cmdtype")
      end
  else
      @warn "[ZMQ Server] Received invalid command $cmdtype"
      resp = Dict{String, String}("status" => "ERROR", "error" => "Command '$cmdtype' not a valid command.")
  end
  return resp
end

function start(zmqServer::ZmqServer; logevents::Bool=false)
    global systemverbs
    global configverbs
    global sessionverbs
    global additionalVerbs
    global plottingVerbs

    # set up a context for zmq
    ctx=Context()
    s1=Socket(ctx, REP)
    ZMQ.bind(s1, "tcp://*:5555")

    drawcounter = 0

    logdir = joinpath("/tmp","Caesar",string(now()))
    if logevents
      mkpath(logdir)
    end

    try
        while zmqServer.isServerActive
            println("waiting to receive...")
            msg = ZMQ.recv(s1)
            logevents ? logEventFile(logdir,msg) : nothing
            out = ZMQ.convert(IOStream, msg)
            str = String(take!(out))

            # start of string processing
            resp = processJsonString(zmqServer, str, drawcounter)

            # transmit the respons back to sender
            ZMQ.send(s1, JSON.json(resp))
        end
    catch ex
        @warn "[ZMQ Server] Something in the zmq/json/rest pipeline broke"
        showerror(stderr, ex, catch_backtrace())
    finally
        @warn "[ZMQ Server] Shutting down server"
        # TODO: Figure out why I can't use Base.isopen here...
        if Base.isopen(s1)
            ZMQ.close(s1)
        end
        ZMQ.close(ctx)
        @warn "[ZMQ Server] Shut down!"
    end
end



function processJsonDir(zmqServer::ZmqServer, jsondir::String)
  global systemverbs
  global configverbs
  global sessionverbs
  global additionalVerbs
  global plottingVerbs

  drawcounter = 0

  # get all json messges
  filenames=sort(glob("*.json",jsondir))
  respdir = joinpath(jsondir,"out")
  mkpath(respdir)

  # process the messages
  for fn in filenames
    # recover request from file
    fid = open(fn, "r")
    @show str = read(fid, String)
    close(fid)
    @show ""
    @show ""

    # process the request
    resp = processJsonString(zmqServer, str, drawcounter)

    # save the response to file
    fid = open(joinpath(respdir,split(fn,'/')[end]), "w")
    write(fid, JSON.json(resp))
    close(fid)
  end

  nothing
end
