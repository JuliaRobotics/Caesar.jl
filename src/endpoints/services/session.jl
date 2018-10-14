export
  addVariable,
  addFactor,
  addOdometry2D,
  addLandmark2D,
  addFactorBearingRangeNormal,
  ls,
  getVert,
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
  varRequest = Unmarshal.unmarshal(VariableRequest, requestDict["variable"])
  varLabel = Symbol(varRequest.label)
  varType = getfield(RoME, Symbol(varRequest.variableType))

  vnext = addNode!(fg, varLabel, varType, N=(isnull(varRequest.N)?100:get(varRequest.N)), ready=0, labels=[varRequest.labels; "VARIABLE"])
  return Dict{String, Any}("status" => "OK", "label" => vnext.label)
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
end

function addFactor(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function addOdometry2D(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function addLandmark2D(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function addFactorBearingRangeNormal(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function ls(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function getVert(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function setReady(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function batchSolve(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function setVarKDE(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function getVarMAPKDE(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function getVarMAPMax(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function getVarMAPMean(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

# Fancy future stuff
function getVarMAPFit(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end
