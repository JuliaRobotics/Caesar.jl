
const respOk = Dict{String, Any}("status" => "ok")

export
    initDfg,
    toggleMockServer,
    registerRobot,
    registerSession

function initDfg(config, dfg, request)::Dict{String, Any}
end

"""
Configure to just return "{"status": "OK"}" for all requests.
"""
function toggleMockServer(config, dfg, request)::Dict{String, Any}
    @show request
    @show config
    if haskey(request, "payload")
        if haskey(request["payload"], "isMockServer")
            config["isMockServer"] = request["payload"]["isMockServer"]
            return Dict{String, Any}("isMockServer" => config["isMockServer"])
        else
            error("Please provide a 'payload' body containing a field 'isMockServer': [\"true\"/\"false\"] for this request.")
        end
    else
        error("Please provide a 'payload' body containing a field 'isMockServer': [\"true\"/\"false\"] for this request.")
    end
end

function registerRobot(config, dfg, request)::Dict{String, Any}
  @info "This is Caesar.registerRobot"
  config["robotId"] = request["robot"]
  return respOk
end

function registerSession(config, dfg, request)::Dict{String, Any}
  @info "This is registerSession"
  if haskey(request, "robot")
    if config["robotId"] == request["robot"]
      config["sessionId"] = request["session"]
      return respOk
    else
      error("Currently only supports one robot, config has $(config["robotId"]), but you asked for robotId=$(request["robot"])")
    end
  end
end
