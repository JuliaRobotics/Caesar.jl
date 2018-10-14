export
  addVariable,
  addFactor,
  addOdometry2D,
  addLandmark2D,
  addFactorBearingRangeNormal,
  setReady,
  batchSolve,
  # per variable
  setVarKDE, # needed for workaround on bad autoinit
  getVarMAPKDE, # marginal belief points (KDE)
  getVarMAPMax, # Future, how many maxes should you get?
  getVarMAPMean,
  # fancy future stuff
  getVarMAPFit # defaul=Normal



function addVariable(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addFactor(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addOdometry2D(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addLandmark2D(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addFactorBearingRangeNormal(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function setReady(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function batchSolve(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function setVarKDE(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function getVarMAPKDE(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function getVarMAPMax(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function getVarMAPMean(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

# Fancy future stuff
function getVarMAPFit(configDict, fg, requestDict)::Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end
