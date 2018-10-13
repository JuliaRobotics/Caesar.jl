
const respOk = Dict{String, Any}("status" => "ok")

export
    initdfg,
    registerRobot,
    registerSession

function initidfg(config, dfg, request)::Dict{String, Any}
end

function registerRobot(config, dfg, request)::Dict{String, Any}
  info("This is Caesar.registerRobot")
  config["robotId"] = request["robot"]
  return respOk
end

function registerSession(config, dfg, request)::Dict{String, Any}
  info("This is registerSession")
  if haskey(request, "robot")
    if config["robotId"] == request["robot"]
      config["sessionId"] = request["session"]
      return respOk
    else
      error("Currently only supports one robot, config has $(config["robotId"]), but you asked for robotId=$(request["robot"])")
    end
  end
end
