
using Distributed
# addprocs(8)

using Caesar
@everywhere using Caesar
using DelimitedFiles, JLD
using IncrementalInference
using KernelDensityEstimatePlotting, RoMEPlotting
using Gadfly, Cairo, Fontconfig
Gadfly.set_default_plot_size(35cm, 25cm)


include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"expDataUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))
include(joinpath(@__DIR__,"plotRangeOnlyUtils.jl"))

expID = "dock"; datawindow = collect(410:421); # 410:450
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
    initManual!(fg,sym,kde!(rand(MvNormal(dposData[i,:],Diagonal([0.5;0.5].^2)),100)))
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
getSolverParams(fg).multiproc = false
getSolverParams(fg).upsolve = true
getSolverParams(fg).downsolve = false
getSolverParams(fg).maxincidence = 100


# writeGraphPdf(fg, show=true)

ensureAllInitialized!(fg)

tree, smt, hist = solveTree!(fg0)
drawTree(tree,filepath = "/tmp/caesar/bt.pdf", show=true, imgs=false)
# fg2 = deepcopy(fg)
# tree, smt, hist = solveTree!(fg,tree)

# plotSASDefault(fg,expID, posData, igt, dposData, datadir=allpaths[1],savedir="/tmp/caesar/plotsas.pdf");
# # plotSASDefault(fg,expID, posData,igt,dposData, datadir=allpaths[1], savedir="/tmp/caesar/test.pdf")
# @async run(`evince /tmp/caesar/plotsas.pdf`)


plotRangeOnlyHandyCompare(fg, expID, igt, posData, dposData, rangewindow, allpaths[1], savefile="/tmp/caesar/upsolve.pdf");



lx = ls(fg, r"x")
lx = sortVarNested(lx)
reverse!(lx)
plotKDE(fg, lx, levels=1)



using Dates

tree = wipeBuildNewTree!(fg)
hists = Dict{Int,Vector{Tuple{DateTime, Int, Function, CliqStateMachineContainer}}}()


ct_l1 = solveCliq!(fg, tree, :l1)
ct_l1 = solveCliq!(fg, tree, :x1)
ct_l1 = solveCliq!(fg, tree, :x11)
ct_l1 = solveCliq!(fg, tree, :x6)
ct_l1 = solveCliq!(fg, tree, :x5)
ct_l1 = solveCliq!(fg, tree, :x8)
ct_l1 = solveCliq!(fg, tree, :x3)
ct_l1 = solveCliq!(fg, tree, :x4)

getSolverParams(fg).downsolve = true

ct_l1 = solveCliq!(fg, tree, :x4)
ct_l1 = solveCliq!(fg, tree, :x3)
ct_l1 = solveCliq!(fg, tree, :x8)
ct_l1 = solveCliq!(fg, tree, :x5)
ct_l1 = solveCliq!(fg, tree, :x6)
ct_l1 = solveCliq!(fg, tree, :x11)
ct_l1 = solveCliq!(fg, tree, :x1)
# ct_l1 = solveCliq!(fg, tree, :l1)


pts = getPoints(getKDE(fg, :l1))
Gadfly.plot(x=pts[1,:],y=pts[2,:], Geom.hexbin())
plotKDE(fg, :l1)


plotKDE(fg, [:x1;:x6;:x11])




stuff = getCliqChildMsgsUp(tree, getClique(tree, :l1), BallTreeDensity)

stuff = IIF.getUpMsgs(getClique(tree, :l1))


plotKDE(fg, :l1)


#PLOTTING ------------
plk= [];
push!(plk,plotBeaconGT(igt));
plotBeaconContours!(plk,fg);

for var in rangewindow
    mysym = Symbol("x$var")
    push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
end


## individual forward convolves
fcts = sort(ls(fg, :l1))
pts = approxConv(fg, fcts[1], :l1)
pl = Gadfly.plot(x=pts[1,:],y=pts[2,:],Geom.histogram2d(xbincount=400, ybincount=400))
savefile = "/tmp/caesar/test.pdf"
pl |> PDF(savefile)

## look at range model for the measurement
mval, = getMeasurements(fg, fcts[2])
Gadfly.set_default_plot_size(35cm,25cm)
Gadfly.plot(y=mval[:],Geom.point)



## all forward convolves
plk = []
LL = BallTreeDensity[];
IncrementalInference.proposalbeliefs!(fg, :l1, map(x->getFactor(fg,x),ls(fg, :l1)), LL, Dict{Int, Vector{BallTreeDensity}}())

for i = 1:length(LL)
    L1ac = getPoints(LL[i])
    push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))
end
savefile = "/tmp/caesar/test.pdf"
Gadfly.plot(plk...) |> PDF(savefile)


LL

LLp = manifoldProduct(LL, Point2().manifolds)
pts = getPoints(LLp)
Gadfly.plot(x=pts[1,:],y=pts[2,:],Geom.hexbin)


@show string(LL[1])

plotKDE(LL[1])
Gadfly.plot(x=getPoints(LL[1])[1,:],y=getPoints(LL[1])[2,:],Geom.hexbin)
pts = getPoints(LL[1])
bw = getBW(LL[1])[:,1]

LL1 = manikde!(pts, [0.2*bw[1];0.1*bw[2]], Point2().manifolds)
plotKDE(LL1)



plotKDE(LL[2])
Gadfly.plot(x=getPoints(LL[2])[1,:],y=getPoints(LL[2])[2,:],Geom.hexbin)
pts = getPoints(LL[2])
bw = getBW(LL[2])[:,1]

LL2 = manikde!(pts, [0.2*bw[1];0.1*bw[2]], Point2().manifolds)
plotKDE(LL2)


LLp12 = manifoldProduct([LL1;LL2], Point2().manifolds)

plotKDE(LLp12)


## product of new forward convolves
ll  = IncrementalInference.predictbelief(fg, :l1, ls(fg,:l1))
plk = []
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



## plot compare full prediction and current result

plk= [];
push!(plk,plotBeaconGT(igt));
# plotBeaconContours!(plk,fg);

for var in rangewindow
    mysym = Symbol("x$var")
    push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
end

L1ac, = predictbelief(fg, :l1,ls(fg, :l1))
push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))

plotKDEMeans!(plk,fg);
push!(plk,plotPath(posData));
push!(plk,plotPath(dposData,colorIn=colorant"blue"));
if expID == "dock"
    push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
elseif expID == "drift"
    push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
end
# plot new contours too
tmp = plotKDE(manikde!(L1ac, Point2().manifolds),levels=3,c=["black"])
push!(plk, tmp.layers[1])
savefile = "/tmp/caesar/test.pdf"
Gadfly.plot(plk...) |> PDF(savefile)
@async run(`evince /tmp/caesar/test.pdf`)












## dev move prior

sortVarNested(ls(fg, r"x"))
lsfPriors(fg)




pX10 = PriorPoint2(MvNormal(posData[10,:],rtkCov))

deleteFactor!(fg, :x11f1)


addFactor!(fg, [:x10], pX10)

writeGraphPdf(fg, show=true)

tree = wipeBuildNewTree!(fg, drawpdf=true)




getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = false
getSolverParams(fg).async = true
getSolverParams(fg).multiproc = false
getSolverParams(fg).upsolve = true
getSolverParams(fg).downsolve = false
getSolverParams(fg).maxincidence = 100


# writeGraphPdf(fg, show=true)

# ensureAllInitialized!(fg)

tree, smt, hist = solveTree!(fg)


plotKDE(fg, reverse(sortDFG(ls(fg, r"x"))), levels=1)

getSolverParams(fg).upsolve = false
getSolverParams(fg).downsolve = true

tree, smt, hist = solveTree!(fg)







using KernelDensityEstimate, KernelDensityEstimatePlotting
using Gadfly
Gadfly.set_default_plot_size(35cm,25cm)





N = 30
x = 0.25 .+ rand(N);
y = x;





ptsd = [x'; y']
ptsx = [x'; 0.01.*randn(N)']
ptsy = [0.01.*randn(N)'; y']

ptsd = [[x; x]'; [y; -y]']

Gadfly.plot(x=ptsd[1,:], y=ptsd[2,:], Geom.hexbin, Coord.Cartesian(fixed=true))


# P1 = manikde!(ptsd, Point2().manifolds)
# P1 = manikde!(ptsd, [0.000001;0.05],Point2().manifolds)


P1 = kde!(ptsd )
P1 = kde!(ptsd [0.000001;0.05])



plotKDE(P1)

|> PDF("/tmp/test.pdf")



getBW(P1)






#
