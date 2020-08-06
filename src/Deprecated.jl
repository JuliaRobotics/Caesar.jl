
export getNode

function getNode(configDict, fg, requestDict)::Dict{String, Any}
    error("Please use getVariable or getFactor, this function is deprecated")
    # TODO: Build a cleaner contract to return this value.
    vert = RoME.getVert(fg, Symbol(requestDict["payload"]))
    return JSON.parse(JSON.json(vert))
end