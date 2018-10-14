export
  addVariable,
  addFactor,
  addOdometry2D,
  addLandmark2D,
  addFactorBearingRangeNormal,
  setReady,
  batchSolve,
  # per variable
  setVarKDE, # needed for workaround on bad autoinit -- sorry
  getVarMAPKDE, # marginal belief points (KDE)
  getVarMAPMax, # Future, how many maxes should you get?
  getVarMAPMean,
  # fancy future stuff
  getVarMAPFit, # defaul=Normal



function addVariable(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addFactor(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addOdometry2D(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addLandmark2D(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function addFactorBearingRangeNormal(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function setReady(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function batchSolve(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function setVarKDE(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function getVarMAPKDE(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function getVarMAPMax(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

function getVarMAPMean(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end

# Fancy future stuff
function getVarMAPFit(configDict, fglFactorGraph, requestDict)Dict{String, Any}
  @show request
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, request)
  error("Not implemented yet!")
end
