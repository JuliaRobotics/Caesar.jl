export
  addVariable,
  addFactor,
  addOdometry2D,
  addLandmark2D,
  addFactorBearingRangeNormal,
  ls,
  getVert,
  setSolvable,
  solveTree!,
  # per variable
  setVarKDE, # needed for workaround on bad autoinit
  getVarMAPKDE, # marginal belief points (KDE)
  getVarMAPMax, # Future, how many maxes should you get?
  getVarMAPMean,
  # fancy future stuff
  getVarMAPFit # defaul=Normal

okResponse = Dict{String, Any}("status" => "OK")

function addVariable(configDict, fg, requestDict)::Dict{String, Any}
  varRequest = Unmarshal.unmarshal(VariableRequest, requestDict["payload"])
  varLabel = Symbol(varRequest.label)
  varType = nothing # TODO type instability here is slow
  try
      varType = getfield(RoME, Symbol(varRequest.variableType))
  catch ex
      io = IOBuffer()
      showerror(io, ex, catch_backtrace())
      err = String(take!(io))
      error("addVariable: Unable to locate variable type '$(varRequest.variableType)'. Please check that it exists in main context. Stack trace = $err")
  end

  @info "Adding variable of type '$(varRequest.variableType)' with id '$(varRequest.label)'..."

  vnext = addVariable!(fg, varLabel, varType, N=(varRequest.N==nothing ? 100 : varRequest.N), solvable=0, labels=Symbol.([varRequest.labels; "VARIABLE"]))
  return Dict{String, Any}("status" => "OK", "id" => vnext.label)
end

function addFactor(configDict, fg, requestDict)::Dict{String, Any}
    if !haskey(requestDict, "payload")
        error("A factor body is required in the request as 'payload' field.")
    end
    factorRequest = requestDict["payload"]
    vars = factorRequest["variables"]
    packedFactor = factorRequest["factor"]
    @info "Adding factor of type '$(factorRequest["factorType"])' to variables '$(vars)'..."

    # Right, carrying on...
    factor = nothing
    try
        factType = _evalType(factorRequest["factorType"])
        factor = convert(factType, packedFactor)
    catch ex
        io = IOBuffer()
        showerror(io, ex, catch_backtrace())
        err = String(take!(io))
        error("addFactor: Unable to convert packed factor data to type '$(factorRequest["factorType"])'. Please check that a converter exists to deserialize '$(factorRequest["factorType"])'. Stack trace = $err")
    end
    f = addFactor!(fg, Symbol.(vars), factor)
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
    if !haskey(requestDict, "payload")
        error("The request does not contain a filter in 'payload' field and this is required for the command")
    end
    lsr = Unmarshal.unmarshal(lsRequest, requestDict["payload"])

    resp = Dict{String, Any}()
    if lsr.variables == "true"
        vars = DistributedFactorGraphs.ls(fg)
        resp["variables"] = map(v -> Dict{String, Any}("id" => v), vars)
    end
    if lsr.factors == "true"
        # Variables
        for vDict in resp["variables"]
            factors = DistributedFactorGraphs.ls(fg, Symbol(vDict["id"]))
            vDict["factors"] = String.(factors)
        end
    end
    return resp
end

function getNode(configDict, fg, requestDict)::Dict{String, Any}
    error("Please use getVariable or getFactor, this function is deprecated")
    # TODO: Build a cleaner contract to return this value.
    vert = RoME.getVert(fg, Symbol(requestDict["payload"]))
    return JSON.parse(JSON.json(vert))
end

function getVariable(configDict, fg, requestDict)::Dict{String, Any}
    variable = DistributedFactorGraphs.getVariable(fg, Symbol(requestDict["payload"]))
    return JSON.parse(JSON.json(variable))
end

function getFactor(configDict, fg, requestDict)::Dict{String, Any}
    factor = DistributedFactorGraphs.getFactor(fg, Symbol(requestDict["payload"]))
    return JSON.parse(JSON.json(factor))
end

function setSolvable(configDict, fg, requestDict)::Dict{String, Any}
    if !haskey(requestDict, "payload")
        error("Request must contain a solvableRequest in a field called 'payload'")
    end
    solvableRequest = requestDict["payload"]
    # Validation of payload
    if !(typeof(solvableRequest["isSolvable"]) <: Int)
        error("setsolvable request must set to solvable to either 0 or 1.")
    end
    if !(solvableRequest["isSolvable"] in [0, 1])
        error("setsolvable request must set to solvable to either 0 or 1.")
    end

    # Action
    if solvableRequest["variables"]!=nothing
        error("setSolvable does not support a request list yet.")
        #TODO: Fix to use getVariable/getFactor for requestDict["variables"]...
    end
    # Do specific variables
    for varLabel in DistributedFactorGraphs.ls(fg)
        v = DistributedFactorGraphs.getVariable(fg, varLabel)
        v.solvable = solvableRequest["isSolvable"]
    end
    for varLabel in DistributedFactorGraphs.lsf(fg)
        v = DistributedFactorGraphs.getFactor(fg, varLabel)
        v.solvable = solvableRequest["isSolvable"]
    end

    # Generate return payload
    return okResponse
end

function solveTree(configDict, fg, requestDict)::Dict{String, Any}
    resp = Dict{String, Any}("startTime" => now())
    # Call solve
    solveTree!(fg)
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
    map = KDE.getKDEMax(getBelief(fg, Symbol(Symbol(requestDict["payload"]))))
    return Dict{String, Any}("MAP" => JSON.parse(JSON.json(map)))
end

function getVarMAPMax(configDict, fg, requestDict)::Dict{String, Any}
    # KDE.getKDEMax(getBelief(fg, :x0))
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Get all KDE modes - Not implemented yet!")
end

function getVarMAPMean(configDict, fg, requestDict)::Dict{String, Any}
    map = KDE.getKDEMean(getBelief(fg, Symbol(Symbol(requestDict["payload"]))))
    return Dict{String, Any}("MAP" => JSON.parse(JSON.json(map)))
end

# Fancy future stuff
function getVarMAPFit(configDict, fg, requestDict)::Dict{String, Any}
  @show requestDict
  # odoFg = Unmarshal.unmarshal(AddOdoFgRequest, requestDict)
  error("Not implemented yet!")
end
