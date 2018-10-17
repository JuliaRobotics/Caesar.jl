
isLoadingPlottingEndpoints = false

try
    getfield(Main, RoMEPlotting)
    # If that works, then we can enable plotting
    isLoadingPlottingEndpoints = true
catch ex
end

# using RoMEPlotting
if isLoadingPlottingEndpoints
    function plotKDE(configDict, fg, requestDict)::Dict{String, Any}
    end

    function plotPose(configDict, fg, requestDict)::Dict{String, Any}
    end

    function drawPoses(configDict, fg, requestDict)::Dict{String, Any}
    end

    function drawPosesLandms(configDict, fg, requestDict)::Dict{String, Any}
    end
end
