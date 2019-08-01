using Caesar, DelimitedFiles, JLD
using IncrementalInference
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

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
            addFactor!(fg, [sym;], pp, autoinit=false)

            dx = dposData[i+1,1] - dposData[i,1];
            dy = dposData[i+1,2] - dposData[i,2];
            p2p2 = Point2Point2(MvNormal([dx;dy],dpσ))
            addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
        elseif  i == window[end]
            pp = PriorPoint2(MvNormal(posData[i,:], rtkCov))
            addFactor!(fg, [sym;], pp, autoinit=false)
        else
            dx = dposData[i+1,1] - dposData[i,1];
            dy = dposData[i+1,2] - dposData[i,2];
            p2p2 = Point2Point2(MvNormal([dx;dy],dpσ))
            addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
        end
    end

    rangewindow = window[1]:rangegap:window[end]
    for i in rangewindow
        sym = Symbol("x$i")
        ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
        addFactor!(fg, [sym;beacon], ppR, autoinit=false)
    end

    writeGraphPdf(fg, engine="neato", filepath=scriptHeader*"fg.pdf")

    getSolverParams(fg).drawtree = false
    getSolverParams(fg).showtree = false

    # tree, smt = batchSolve!(fg,maxparallel=100)

    tree, smt, hist = solveTree!(fg, maxparallel=100)
    drawTree(tree, filepath=scriptHeader*"bt.pdf")

    plotSASDefault(fg,posData,igt,datadir=allpaths[1],savedir=scriptHeader*"SASdefault.pdf")

    #RangeOnly PLOTTING ------------
    plk= [];
    push!(plk,plotBeaconGT(iGTtemp));
    plotBeaconContours!(plk,fg);

    for var in rangewindow
        mysym = Symbol("x$var")
        push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
    end

    # for var in window       #Plot only for range factors
    #     sym = Symbol("x$var")
    #     global onetime
    #     for mysym in ls(fg,sym)
    #         if occursin(r"l1",string(mysym))
    #             if var > 130 && onetime       #Plot one or more approxConv
    #                 L1ac = approxConv(fg,mysym,:l1)
    #                 # K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=3,layers=true)
    #                 # push!(plk,K1...)
    #                 push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
    #                 # push!(pkde,plotKDE(kde!(L1ac),layers=true)...)
    #                 onetime = false
    #                 # push!(plk,Gadfly.Theme(key_position = :none));
    #                 # push!(plk, Guide.xlabel("X (m)"), Guide.ylabel("Y (m)"))
    #
    #                 X1 = getKDEMean(getVertKDE(fg,sym))
    #                 push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"magenta",point_size=1.5pt,highlight_width=0pt)))
    #             end
    #             X1 = getKDEMean(getVertKDE(fg,sym))
    #             push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"red",point_size=1.5pt,highlight_width=0pt)))
    #         end
    #     end
    # end

    plotKDEMeans!(plk,fg);
    push!(plk,plotPath(posData));

    if expID == "dock"
        push!(pltemp, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
    elseif expID == "drift"
        push!(pltemp, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
    end
    savefile = scriptHeader*"showRanges.pdf"
    Gadfly.plot(plk...) |> PDF(savefile)
end
