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
  varType = nothing
  try
      varType = getfield(RoME, Symbol(varRequest.variableType))
  catch ex
      io = IOBuffer()
      showerror(io, ex, catch_backtrace())
      err = String(take!(io))
      error("addVariable: Unable to locate variable type '$(varRequest.variableType)'. Please check that it exists in main context. Stack trace = $err")
  end

  info("Adding variable of type '$(varRequest.variableType)' with id '$(varRequest.label)'...")

  vnext = addNode!(fg, varLabel, varType, N=(isnull(varRequest.N)?100:get(varRequest.N)), ready=0, labels=[varRequest.labels; "VARIABLE"])
  return Dict{String, Any}("status" => "OK", "id" => vnext.label)
end

function addFactor(configDict, fg, requestDict)::Dict{String, Any}
    if !haskey(requestDict, "factor")
        error("A factor body is required in the request.")
    end
    info("Adding factor of type '$(requestDict["factor"]["factorType"])' to variables '$(requestDict["factor"]["variables"])'...")

    # Right, carrying on...
    @show factType = _evalType(requestDict["factor"]["factorType"])
    factor = nothing
    try
        @show factor = convert(factType, requestDict["factor"])
    catch ex
        io = IOBuffer()
        showerror(io, ex, catch_backtrace())
        err = String(take!(io))
        error("addFactor: Unable to convert packed factor data to type '$factType'. Please check that a converter exists to deserialize '$factType'. Stack trace = $err")
    end
    f = addFactor!(fg, Symbol.(requestDict["factor"]["variables"]), factor)
    return Dict{String, Any}("status" => "OK", "id" => f.label)
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
        error("The request does not contain a filter parameter and this is required for the command")
    end
    lsr = Unmarshal.unmarshal(lsRequest, requestDict["filter"])

    resp = Dict{String, Any}()
    if lsr.variables == "true"
        vars = Caesar.ls(fg)
        resp["variables"] = map(v -> Dict{String, Any}("id" => v), vars[1])
    end
    if lsr.factors == "true"
        # Variables
        for vDict in resp["variables"]
            factors = Caesar.ls(fg, Symbol(vDict["id"]))
            vDict["factors"] = String.(factors)
        end
    end
    return resp
end

function getNode(configDict, fg, requestDict)::Dict{String, Any}
    # TODO: Build a cleaner contract to return this value.
    vert = RoME.getVert(fg, Symbol(requestDict["id"]))
    return JSON.parse(JSON.json(vert))
end

function setReady(configDict, fg, requestDict)::Dict{String, Any}
    @show requestDict

    # Unmarshal if necessary
    if !haskey(requestDict, "params")
        error("Request must contain a ReadyRequest in a field called 'param'")
    end
    readyRequest = Unmarshal.unmarshal(SetReadyRequest, requestDict["params"])
    # Validation of payload

    # Action
    varLabels = isnull(readyRequest.variables) ? union(Caesar.ls(fg)...) : Symbol.(get(requestDict.variables))
    # Do specific variables
    for varLabel in varLabels
        v = getVert(fg, varLabel)
        v.attributes["ready"] = readyRequest.isReady
    end

    # Generate return payload
    return okResponse
end

function batchSolve(configDict, fg, requestDict)::Dict{String, Any}
    resp = Dict{String, Any}("startTime" => now())
    # Call solve
    batchSolve!(fg)
    resp["endTime"] = now()
    resp["durationSec"] = Dates.value(resp["endTime"] - resp["startTime"])/1000.0
    resp["status"] = "OK"
    return resp
end

function setVarKDE(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end

function getVarMAPKDE(configDict, fg, requestDict)::Dict{String, Any}
    map = KDE.getKDEMax(getVertKDE(fg, Symbol(Symbol(requestDict["id"]))))
    return Dict{String, Any}("MAP" => JSON.parse(JSON.json(map)))
end

function getVarMAPMax(configDict, fg, requestDict)::Dict{String, Any}
    # KDE.getKDEMax(getVertKDE(fg, :x0))
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Get all KDE modes - Not implemented yet!")
end

function getVarMAPMean(configDict, fg, requestDict)::Dict{String, Any}
    map = KDE.getKDEMean(getVertKDE(fg, Symbol(Symbol(requestDict["id"]))))
    return Dict{String, Any}("MAP" => JSON.parse(JSON.json(map)))
end

# Fancy future stuff
function getVarMAPFit(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end
