
# Set of standard plotting functions for Kayaks / SAS
# For mix and match purposes these functions return Gadfly layers except for default plot


function plotSASDefault(fg, posData::Array, expnum::Int ; datadir::String=joinpath(ENV["HOME"],"data", "kayaks", "08_10_parsed") , savedir::String="/tmp/test.pdf")
    plk = [];
    push!(plk,plotBeaconGT(expnum,datadir=datadir));
    plotMeans!(plk,fg);
    push!(plk,plotPath(posData));
    if expnum == 1
        push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-150, ymax=30,fixed=true))
    elseif expnum ==2
        push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
    end
    plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")
end

function plotMeans!(plk,fg;regx::Regex=r"x")
    for sym in ls(fg)
        if occursin(regx,string(sym))
            xData = getKDEMean(getVertKDE(fg,sym))
            push!(plk, plotPoint(xData))
        end
    end
end

function plotPoint(posData::Array)
    return layer(x=[posData[1];],y=[posData[2];],Geom.point,Theme(default_color=colorant"blue",point_size = 1.5pt,highlight_width = 0pt))
end

function plotPoints(pos::Array;colorIn::String="blue")
        return layer(x=posData[:,1],y=posData[:,2],Geom.point,Theme(default_color=colorant"blue",point_size = 1.5pt,highlight_width = 0pt))
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
