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

okResponse = Dict{String, Any}("status" => "OK")

function addVariable(configDict, fg, requestDict)::Dict{String, Any}
  varRequest = Unmarshal.unmarshal(VariableRequest, requestDict["variable"])
  varLabel = Symbol(varRequest.label)
  varType = getfield(RoME, Symbol(varRequest.variableType))

  vnext = addNode!(fg, varLabel, varType, N=(isnull(varRequest.N)?100:get(varRequest.N)), ready=0, labels=[varRequest.labels; "VARIABLE"])
  return Dict{String, Any}("status" => "OK", "id" => vnext.label)
end

function addFactor(configDict, fg, requestDict)::Dict{String, Any}
    if !haskey(requestDict, "factor")
        error("A factor body is required in the request.")
    end
    # Handling the weird overflow issue with measurements
    measurement = requestDict["factor"]["measurement"]
    requestDict["factor"]["measurement"] = []
    @show factorRequest = Unmarshal.unmarshal(FactorRequest, requestDict["factor"])
    factorRequest.measurement = measurement

    # Right, carrying on...
    @show factorRequest
    return okResponse
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
    if !haskey(requestDict, "filter")
        error("The reques does not contain a filter parameter and this is required for the command")
    end
    lsRequest = Unmarshal.unmarshal(Caesar.lsRequest, requestDict["filter"])

    resp = Dict{String, Any}()
    if lsRequest.variables == "true"
        vars = ls(fg)
        @show vars
        resp["variables"] = map(v -> Dict{String, Any}("id" => v), vars[1])
    end
    if lsRequest.factors == "true"
        # Variables
        for vDict in resp["variables"]
            factors = lsf(fg, Symbol(vDict["id"]))
            @show factors
            vDict["factors"] = String.(factors)
        end
    end
    return resp
end

function getNode(configDict, fg, requestDict)::Dict{String, Any}
    # TODO: Build a cleaner contract to return this value.
    return RoME.getVert(fg, Symbol(requestDict["id"]))
end

function setReady(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function batchSolve(configDict, fg, requestDict)::Dict{String, Any}
    resp = Dict{String, Any}("startTime" => now())
    # Call solve
    batchSolve(fg)
    resp["endTime"] = now()
    resp["durationSec"] = Dates.value(resp["endTime"] - resp["startTime"])/1000.0
    return resp
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
