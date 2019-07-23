
# Set of standard plotting functions for Kayaks / SAS
# For mix and match purposes these functions return Gadfly layers except for default plot


function plotSASDefault(fg, posData::Array, expnum::Int ; datadir::String=joinpath(ENV["HOME"],"data", "kayaks", "08_10_parsed") , savedir::String="/tmp/test.pdf")
    pltemp = [];
    push!(pltemp,plotBeaconGT(expnum,datadir=datadir));
    push!(pltemp,plotBeaconSolve(fg));
    push!(pltemp,plotPath(posData));
    plotMeans!(pltemp,fg);
    if expnum == 1
        push!(pltemp, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
    elseif expnum ==2
        push!(pltemp, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
    end
    push!(pltemp,Gadfly.Theme(key_position = :none));
    push!(pltemp,Gadfly.Guide.xlabel("X (m)"),Gadfly.Guide.ylabel("Y (m)"));
    pltempplot = Gadfly.plot(pltemp...); pltempplot |> PDF("/tmp/test.pdf")
end

function plottoPDF(layersin)
    pltempplot = Gadfly.plot(layersin...); pltempplot |> PDF("/tmp/test.pdf")
end

function plotBeaconSolve(fg)
    L1 = getVal(getVariable(fg, :l1))
    # K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
    # push!(pltemp,K1...)
    # push!(pltemp,Gadfly.Theme(key_position = :none));
    return layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300))
end

function plotMeans!(pltemp,fg;regx::Regex=r"x")
    for sym in ls(fg)
        if occursin(regx,string(sym))
            xData = getKDEMean(getVertKDE(fg,sym))
            push!(pltemp, plotPoint(xData))
        end
    end
end

function plotPoint(posData::Array)
    return layer(x=[posData[1];],y=[posData[2];],Geom.point,Theme(default_color=colorant"blue",point_size = 1.25pt,highlight_width = 0pt))
end

function plotPoints(pos::Array;colorIn::String="blue")
        return layer(x=posData[:,1],y=posData[:,2],Geom.point,Theme(default_color=colorant"blue",point_size = 1.25pt,highlight_width = 0pt))
end

function plotPath(pos::Array)
    return layer(x=posData[:,1],y=posData[:,2], Geom.path,Theme(default_color=colorant"green"))
end

function plotBeaconGT(expnum::Int; datadir::String=joinpath(ENV["HOME"],"data", "kayaks", "08_10_parsed"))
    if expnum==1
        igt = [17.0499;1.7832];

        return layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt));
    else
        ijldname = datadir * "/exp$expnum" * ".jld"
        iload = load(ijldname)
        igt = iload["icarus_gt"]
        windowstart = iload["ibegin"];
        windowend = iload["iend"];
        return layer(x=igt[:,1],y=igt[:,2], label=String["Beacon Ground Truth";],Geom.path,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt));
    end
end
