
# Set of standard plotting functions for Kayaks / SAS
# For mix and match purposes these functions return Gadfly layers except for default plot


function plotSASDefault(fg, expID::String, posData::Array, iGTtemp::Array, dposData::Array ; datadir::String=joinpath(ENV["HOME"],"data", "kayaks", "08_10_parsed") , savedir::String="/tmp/test.pdf")
    pltemp = [];
    push!(pltemp,plotBeaconGT(iGTtemp));
    push!(pltemp,plotBeaconMax(fg));
    push!(pltemp,plotBeaconMean(fg));
    push!(pltemp,plotBeaconHist(fg));
    push!(pltemp,Gadfly.Theme(key_position = :none));
    plotPriors!(pltemp,fg);
    plotKDEMaxs!(pltemp,fg);
    push!(pltemp,plotPath(posData));
    push!(pltemp,plotPath(dposData,colorIn=colorant"steelblue1"));
    plotBeaconContours!(pltemp,fg);

    if expID == "dock"
        push!(pltemp, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
    elseif expID == "drift"
        push!(pltemp, Coord.cartesian(xmin=20, xmax=200, ymin=-260, ymax=0,fixed=true))
    end
    # push!(pltemp,Gadfly.Guide.xlabel("X (m)"),Gadfly.Guide.ylabel("Y (m)"));
    pltempplot = Gadfly.plot(pltemp...); pltempplot |> PDF(savedir)
end

function plotPriors!(plHolderIn,fg)
    for mysym in ls(fg)
        for myfac in ls(fg,mysym)
            if occursin(r"^x\df",string(myfac))
                xData = getKDEMax(getVertKDE(fg,mysym))
                push!(plHolderIn,plotPoint(xData,colorIn=colorant"green2"))
            end
        end
    end
end

function pltoPDF(plHolderIn ; savedir::String="/tmp/test.pdf")
    Gadfly.plot(plHolderIn...) |> PDF(savedir)
end

function plotBeaconMax(fg)
    L1 = getKDEMax(getVertKDE(fg, :l1))
    return layer(x=[L1[1];],y=[L1[2];], label=String["Beacon Max";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"darkorange",highlight_width = 0pt));
end

function plotBeaconMean(fg)
    L1 = getKDEMean(getVertKDE(fg, :l1))
    return layer(x=[L1[1];],y=[L1[2];], label=String["Beacon Mean";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"orangered",highlight_width = 0pt));
end

function plotBeaconHist(fg)
    L1 = getVal(getVariable(fg, :l1))
    return layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300))
end

function plotBeaconContours!(plHolderIn, fg)
    K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
    push!(plHolderIn,K1...)
    push!(plHolderIn,Gadfly.Theme(key_position = :none));
end

function plotKDEMaxs!(plHolderIn,fg;regx::Regex=r"x")
    for sym in ls(fg)
        if occursin(regx,string(sym))
            xData = getKDEMax(getVertKDE(fg,sym))
            push!(plHolderIn, plotPoint(xData))
        end
    end
end

function plotKDEMeans!(plHolderIn,fg;regx::Regex=r"x")
    for sym in ls(fg)
        if occursin(regx,string(sym))
            xData = getKDEMean(getVertKDE(fg,sym))
            push!(plHolderIn, plotPoint(xData))
        end
    end
end

function plotPoint(pointIn::Array; colorIn = colorant"blue")
    return layer(x=[pointIn[1];],y=[pointIn[2];],Geom.point,Theme(default_color=colorIn,point_size = 1.25pt,highlight_width = 0pt))
end

function plotPoints(pointsIn::Array;colorIn::String="blue")
        return layer(x=pointsIn[:,1],y=pointsIn[:,2],Geom.point,Theme(default_color=colorant"blue",point_size = 1.25pt,highlight_width = 0pt))
end

function plotPath(pathIn::Array; colorIn = colorant"green")
    return layer(x=pathIn[:,1],y=pathIn[:,2], Geom.path,Theme(default_color=colorIn))
end

function plotBeaconGT(iGTtemp::Array)
    if size(iGTtemp,2) ==  1
        return layer(x=[iGTtemp[1];],y=[iGTtemp[2];], label=String["Beacon GT";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt));
    else
        return layer(x=iGTtemp[:,1],y=iGTtemp[:,2], label=String["Beacon GT";],Geom.path,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt));
    end
end
