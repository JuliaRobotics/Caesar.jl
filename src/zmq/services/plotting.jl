
isLoadingPlottingEndpoints = false
try
    getfield(Main, :RoMEPlotting)
    # If that works, then we can enable plotting
    isLoadingPlottingEndpoints = true
catch ex
end

if isLoadingPlottingEndpoints
    @info "[ZMQ Server] Plotting endpoints are enabled!"

    function plotKDE(configDict, fg, requestDict)::Dict{String, Any}
        error("Not implemented yet!")
    end

    function plotPose(configDict, fg, requestDict)::Dict{String, Any}
        error("Not implemented yet!")
    end

    function drawPoses(configDict, fg, requestDict)::Dict{String, Any}
        plotRequest = Unmarshal.unmarshal(PlotRequest, requestDict["plotParams"])
        pl = RoMEPlotting.drawPoses(fg);
        io = IOBuffer();
        Gadfly.draw(PNG(io, (plotRequest.widthPx)px, (plotRequest.heightPx)px), pl);
        return Dict{String, Any}("status" => "ok", "mimetype" => "image/png", "encoding" => "none", "data" => io.data)
    end

    function drawPosesLandms(configDict, fg, requestDict)::Dict{String, Any}
        plotRequest = Unmarshal.unmarshal(PlotRequest, requestDict)
        pl = RoMEPlotting.drawPosesLandms(fg);
        io = IOBuffer();
        Gadfly.draw(PNG(io, (plotRequest.widthPx)px, (plotRequest.heightPx)px), pl);
        return Dict{String, Any}("status" => "ok", "mimetype" => "image/png", "encoding" => "none", "data" => io.data)
    end
else
    @info "[ZMQ Server] Plotting endpoints are disabled! Please call 'using RoMEPlotting' before 'using Caesar' if you would like to enable them."
end
