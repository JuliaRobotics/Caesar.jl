#

export
  registerRobot,
  registerSession,
  addNode,
  addFactor


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


function registerRobot(config, request)::String
  info("This is Caesar.registerRobot")
  config["robotId"] = request["robot"]
  return ""
end

function registerSession(config, request)::String
  info("This is registerSession")
  if haskey(request, "robot")
    if config["robotId"] == request["robot"]
      config["sessionId"] = request["session"]
      return ""
    else
      str = "Currently only supports one robot, config has $(config["robotId"]), but you asked for robotId=$(request["robot"])"
      warn(str)
      return str
    end
  end
end


# struct addPose3
#
# end

function addVariable(config::Dict, fgl::FactorGraph, request::Dict)::String
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  return "Not implemented yet!"
end

function addFactor(config::Dict, fgl::FactorGraph, request::Dict)::String
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  return "Not implemented yet!"
end
