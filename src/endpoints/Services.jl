#

export
  addVariable,
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

# struct addPose3
#
# end

function addVariable(config::Dict, fgl::FactorGraph, request::Dict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addFactor(config::Dict, fgl::FactorGraph, request::Dict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end
