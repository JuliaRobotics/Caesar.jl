using Distributed
addprocs(8)

using Caesar
# @everywhere using Caesar
using DelimitedFiles, JLD
using IncrementalInference
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"expDataUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))

expID = "dock"; datawindow = collect(410:450);
# expID = "drift"; datawindow = collect(1550:1750);

allpaths = getPaths(expID,"range", trialID = 1);
igt = loadGT(datawindow,expID,allpaths);
mfin = loadRanges(datawindow,allpaths)
posData = loadNav(datawindow,allpaths)
dposData = deepcopy(posData)

dposData = deepcopy(posData)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

window = 1:datawindow[end]-datawindow[1];
fg = initfg();
beacon = :l1
addVariable!(fg, beacon, Point2 )

for i in window
    sym = Symbol("x$i")
    addVariable!(fg, sym, Point2)
    manualinit!(fg,sym,kde!(rand(MvNormal(dposData[i,:],Diagonal([0.5;0.5].^2)),100)))
end

rtkCov = Matrix(Diagonal([0.1;0.1].^2));
dpσ = Matrix(Diagonal([0.2;0.2].^2))

for i in window
    sym = Symbol("x$i")
    nextsymi = i+1;
    nextsym = Symbol("x$nextsymi")

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

rangewindow = window[1]:5:window[end]
for i in rangewindow
    sym = Symbol("x$i")
    ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
    addFactor!(fg, [sym;beacon], ppR, autoinit=false)
end

writeGraphPdf(fg,viewerapp="", engine="neato", filepath = "/tmp/test.pdf")

getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = false
getSolverParams(fg).async = true
getSolverParams(fg).multiproc = true
getSolverParams(fg).downsolve = true


tree, smt, hist = solveTree!(fg, maxparallel=100)
drawTree(tree,filepath = "/tmp/test.pdf")
# fg2 = deepcopy(fg)
# tree, smt, hist = solveTree!(fg,tree,maxparallel=100)

# plotSASDefault(fg,expID, posData,igt,datadir=allpaths[1],savedir=scriptHeader*"SASdefault.pdf")
plotSASDefault(fg,expID, posData,igt,dposData, datadir=allpaths[1], savedir="/tmp/caesar/test.pdf")
run(`evince /tmp/caesar/test.pdf`)


#PLOTTING ------------

plk= [];
push!(plk,plotBeaconGT(igt));
plotBeaconContours!(plk,fg);

for var in rangewindow
    mysym = Symbol("x$var")
    push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
end

# all forward convolves
LL = BallTreeDensity[];
IncrementalInference.proposalbeliefs!(fg, :l1, map(x->getFactor(fg,x),ls(fg, :l1)), LL, Dict{Int, Vector{BallTreeDensity}}())

# for i = 1:length(LL)
    # L1ac = rand(LL[i],100)
    # push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))
# end

# product of new forward convolves
ll  = IncrementalInference.predictbelief(fg, :l1, ls(fg,:l1))
L1ac = ll[1];
push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=200, ybincount=200)))
# L1est = manikde(ll, Point2().manifolds)

plotKDEMeans!(plk,fg);
push!(plk,plotPath(dposData,colorIn=colorant"blue"));
push!(plk,plotPath(posData));
if expID == "dock"
    push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
elseif expID == "drift"
    push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
end
savefile = "/tmp/caesar/test.pdf"
Gadfly.plot(plk...) |> PDF(savefile)

plk= [];
push!(plk,plotBeaconGT(igt));
plotBeaconContours!(plk,fg);

for var in rangewindow
    mysym = Symbol("x$var")
    push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
end

L1ac = predictbelief(fg, :l1,ls(fg, :l1))
push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))

plotKDEMeans!(plk,fg);
push!(plk,plotPath(posData));
push!(plk,plotPath(dposData,colorIn=colorant"blue"));
if expID == "dock"
    push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
elseif expID == "drift"
    push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
end
savefile = scriptHeader*"predBel.pdf"
Gadfly.plot(plk...) |> PDF(savefile)
