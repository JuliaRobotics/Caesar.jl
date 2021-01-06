using Caesar, DelimitedFiles, JLD
using IncrementalInference
using Gadfly, Cairo, Fontconfig
using KernelDensityEstimatePlotting

include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))
include(joinpath(@__DIR__,"expDataUtils.jl"))

function main(expID::String, rangegap::Int, wstart::Int, wend::Int, trialID::Int)

    datawindow = collect(wstart:wend);
    allpaths = getPaths(expID,"range", currloc = "lj", trialID = trialID);
    scriptHeader = joinpath(allpaths[3],expID*"_rgap$(rangegap)_trial$(trialID)_f$(wstart)t$(wend)_")

    igt = loadGT(datawindow,expID,allpaths);
    mfin = loadRanges(datawindow,allpaths)
    posData = loadNav(datawindow,allpaths)

    dposData = deepcopy(posData)
    cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

    window = 1:datawindow[end]-datawindow[1];
    fg = initfg();
    beacon = :l1
    addVariable!(fg, beacon, Point2 )

    for i in window
        sym = Symbol("x$i")
        addVariable!(fg, sym, Point2)
    end

    rtkCov = Matrix(Diagonal([0.1;0.1].^2));
    dpσ = Matrix(Diagonal([0.2;0.2].^2))

    for i in window
        sym = Symbol("x$i")
        nextsym = Symbol("x$(i+1)")

        if i == window[1]
            pp = PriorPoint2(MvNormal(posData[i,:], rtkCov))
            addFactor!(fg, [sym;], pp, autoinit=true)

            dx = dposData[i+1,1] - dposData[i,1];
            dy = dposData[i+1,2] - dposData[i,2];
            p2p2 = Point2Point2(MvNormal([dx;dy],dpσ))
            addFactor!(fg, [sym;nextsym], p2p2, autoinit=true)
        elseif  i == window[end]
            pp = PriorPoint2(MvNormal(posData[i,:], rtkCov))
            addFactor!(fg, [sym;], pp, autoinit=false)
        else
            dx = dposData[i+1,1] - dposData[i,1];
            dy = dposData[i+1,2] - dposData[i,2];
            p2p2 = Point2Point2(MvNormal([dx;dy],dpσ))
            addFactor!(fg, [sym;nextsym], p2p2, autoinit=true)
        end
    end

    rangecounter = 0;
    rangewindow = window[1]:rangegap:window[end]
    for i in rangewindow
        sym = Symbol("x$i")
        ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
        addFactor!(fg, [sym;beacon], ppR, autoinit=false)
        rangecounter += 1;
    end

    # writeGraphPdf(fg,viewerapp="", engine="neato", filepath=scriptHeader*"fg.pdf")

    getSolverParams(fg).drawtree = false
    getSolverParams(fg).showtree = false
    getSolverParams(fg).limititers=500
    getSolverParams(fg).maxincidence = 400


    tree, smt, hist = solveTree!(fg)
    println("Ended Script with $(rangecounter) range factors \n")

    # tree, smt, hist = solveTree!(fg)
    # drawTree(tree, filepath=scriptHeader*"bt.pdf")

    # plotSASDefault(fg,expID, posData,igt,dposData,datadir=allpaths[1],savedir=scriptHeader*"SASdefault.pdf")

    #RangeOnly PLOTTING ------------
    # plk= [];
    # push!(plk,plotBeaconGT(igt));
    # push!(plk,plotBeaconMax(fg));
    # push!(plk,plotBeaconMean(fg));
    #
    # for var in rangewindow
    #     mysym = Symbol("x$var")
    #     push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
    # end
    #
    # # counter = 1;
    # for var in rangewindow       #Plot only for range factors
    #     sym = Symbol("x$var")
    #     for mysym in ls(fg,sym)
    #         if occursin(r"l1",string(mysym))
    #             # if counter < 4       #Plot one or more approxConv
    #                 L1ac = approxConv(fg,mysym,:l1)
    #                 # K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=3,layers=true)
    #                 # push!(plk,K1...)
    #                 push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))
    #                 # push!(pkde,plotKDE(kde!(L1ac),layers=true)...)
    #                 # onetime = false
    #                 # push!(plk,Gadfly.Theme(key_position = :none));
    #                 # push!(plk, Guide.xlabel("X (m)"), Guide.ylabel("Y (m)"))
    #                 # counter +=1;
    #             # end
    #
    #         end
    #     end
    # end
    #
    # plotKDEMeans!(plk,fg);
    # push!(plk,plotPath(posData));
    # push!(plk,plotPath(dposData,colorIn=colorant"blue"));
    # plotBeaconContours!(plk,fg);
    #
    # if expID == "dock"
    #     push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
    # elseif expID == "drift"
    #     push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
    # end
    # savefile = scriptHeader*"showRanges.pdf"
    # Gadfly.plot(plk...) |> PDF(savefile)
    #
    # plk= [];
    # push!(plk,plotBeaconGT(igt));
    # # push!(plk,plotBeaconMax(fg));
    # # push!(plk,plotBeaconMean(fg));
    # L1pd = predictbelief(fg, :l1,ls(fg, :l1))
    # L1ac = L1pd[1];
    #
    # L1 = getKDEMax(kde!(L1ac))
    # push!(plk,layer(x=[L1[1];],y=[L1[2];], label=String["Beacon Max";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"darkorange",highlight_width = 0pt)));
    #
    # L1 = getKDEMean(kde!(L1ac))
    # push!(plk,layer(x=[L1[1];],y=[L1[2];], label=String["Beacon Mean";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"orangered",highlight_width = 0pt)))
    #
    # for var in rangewindow
    #     mysym = Symbol("x$var")
    #     push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
    # end
    #
    # push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
    #
    # K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
    # push!(plk,K1...)
    # push!(plk,Gadfly.Theme(key_position = :none));
    #
    # plotKDEMeans!(plk,fg);
    # push!(plk,plotPath(posData));
    # push!(plk,plotPath(dposData,colorIn=colorant"blue"));
    # # plotBeaconContours!(plk,fg);
    # if expID == "dock"
    #     push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
    # elseif expID == "drift"
    #     push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
    # end
    # savefile = scriptHeader*"predBel.pdf"
    # Gadfly.plot(plk...) |> PDF(savefile)
    #
    # if expID == "dock"
    #     l1fit = getKDEMean(getBelief(fg,:l1))
    #     meanerror = sqrt((igt[1]-l1fit[1])^2+(igt[2]-l1fit[2])^2)
    #
    #     l1max = getKDEMax(getBelief(fg,:l1))
    #     maxerror = sqrt((igt[1]-l1max[1])^2+(igt[2]-l1max[2])^2)
    #
    #     mykde = getBelief(fg,:l1)
    #     mynorm = kde!(rand(fit(MvNormal,getVal(fg,:l1)),200))
    #     kld = min(abs(KernelDensityEstimate.kld(mynorm,mykde)),abs(KernelDensityEstimate.kld(mykde,mynorm)))
    # elseif expID == "drift"
    #     l1fit = getKDEMean(getBelief(fg,:l1))
    #     meanerror = sqrt((mean(igt[:,1])-l1fit[1])^2+(mean(igt[:,2])-l1fit[2])^2)
    #
    #     l1max = getKDEMax(getBelief(fg,:l1))
    #     maxerror = sqrt((mean(igt[:,1])-l1max[1])^2+(mean(igt[:,2])-l1max[2])^2)
    #
    #     mykde = getBelief(fg,:l1)
    #     mynorm = kde!(rand(fit(MvNormal,getVal(fg,:l1)),200))
    #     kld = min(abs(KernelDensityEstimate.kld(mynorm,mykde)),abs(KernelDensityEstimate.kld(mykde,mynorm)))
    # end
    # writedlm(scriptHeader*"stats.txt", [meanerror maxerror kld], ",")
end
