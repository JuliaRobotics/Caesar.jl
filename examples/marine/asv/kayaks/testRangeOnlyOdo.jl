using Revise

using Caesar
using KernelDensityEstimatePlotting
using KernelDensityEstimate, IncrementalInference
using DocStringExtensions, DelimitedFiles
using Gadfly, Cairo, Fontconfig
Gadfly.set_default_plot_size(35cm,25cm)

include(joinpath(@__DIR__,"slamUtils.jl"))

wstart = 415;
wlen = 171;

topdir = joinpath(ENV["HOME"],"data","kayaks")
trialstr = "20_gps_pos";
datawindow = collect(wstart:wstart+wlen-1);
datadir = joinpath(topdir,trialstr);
rangedir = joinpath(topdir, "rangeOnly_"*trialstr)

mfin = zeros(7501,2,wlen);
posData = zeros(wlen,2);
for i in 1:wlen
    dataindex = datawindow[i];
    navfile = datadir*"/nav$(dataindex).csv"
    posData[i,:] = readdlm(navfile,',',Float64,'\n')
    rangeFile = rangedir*"/range$(dataindex).txt";
    mfin[:,:,i] = readdlm(rangeFile,',',Float64,'\n')
end

dposData = deepcopy(posData)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

fg = initfg();

poses = [Symbol("x$i") for i in 1:wlen]
for sym in poses
  addVariable!(fg, sym, Point2)
end

priors = [1,wlen]
rtkCov = Matrix(Diagonal([0.1;0.1].^2))
#Priors
for i in priors
    pp = PriorPoint2(MvNormal(posData[i,:], rtkCov))
    addFactor!(fg, [Symbol("x$i");], pp, autoinit=true)
end

vps = 1:wlen-1
dpσ = Matrix(Diagonal([0.2;0.2].^2))
for i in vps
    dx = dposData[i+1,1] - dposData[i,1];
    dy = dposData[i+1,2] - dposData[i,2];
    p2p2 = Point2Point2(MvNormal([dx;dy],dpσ))
    addFactor!(fg, [Symbol("x$i");Symbol("x$(i+1)")], p2p2, autoinit=true)
end

beacon = :l1
addVariable!(fg, beacon, Point2 )
# manualinit!(fg,beacon,kde!(rand(MvNormal([0;0],Matrix(Diagonal([7.0;7].^2))),100)))

rangewindow = 1:3:wlen
for i in rangewindow
    sym = Symbol("x$i")
    ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
    addFactor!(fg, [sym;beacon], ppR, autoinit=false)
end

# writeGraphPdf(fg,viewerapp="", engine="neato", filepath="/tmp/fg.pdf")
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = false
getSolverParams(fg).dbg = true


ensureAllInitialized!(fg)
tree, smt, hist = solveTree!(fg,maxparallel=200, recordcliqs=[:x19;:x169;:x166;:x134;:x171])

fetchCliqTaskHistoryAll!(smt, hist)
drawTree(tree, show=true)

#Plotting

plk= [];
for sym in poses[end-5:end]  #plot last 5 poses with contour
    X1 = getKDEMean(getVertKDE(fg,sym))
    push!(plk, layer(x=[X1[1];],y=[X1[2];], label=["$(sym)";], Geom.point, Geom.label, Theme(default_color=colorant"blue",point_size = 1.5pt,highlight_width = 0pt)))
    K1 = plotKDEContour(getVertKDE(fg,sym),xlbl="", ylbl="",levels=2,layers=true);
    push!(plk,K1...)
    push!(plk,Gadfly.Theme(key_position = :none));
end
for sym in poses #plotting all syms no labels
    X1 = getKDEMean(getVertKDE(fg,sym))
    push!(plk, layer(x=[X1[1];],y=[X1[2];], label=["$(sym)";], Geom.point, Theme(default_color=colorant"blue",point_size = 1.5pt,highlight_width = 0pt)))
    # K1 = plotKDEContour(getVertKDE(fg,sym),xlbl="", ylbl="",levels=2,layers=true);
    # push!(plk,K1...)
    # push!(plk,Gadfly.Theme(key_position = :none));
end
push!(plk,layer(x=posData[:,1],y=posData[:,2], Geom.path, Theme(default_color=colorant"green2")))
push!(plk,layer(x=dposData[:,1],y=dposData[:,2], Geom.path, Theme(default_color=colorant"blue2")))

# push!(plk, Coord.cartesian(xmin=-10, xmax=120, ymin=-120, ymax=-30,fixed=true)) #full plot
push!(plk, Coord.cartesian(xmin=95, xmax=105, ymin=-81, ymax=-70,fixed=true)) #crop plot to last points
plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf");


# igt = [17.0499;1.7832];
#
# push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));
#
# X1 = approxConv(fg,:x4l1f1,:l1,N=200)
# push!(plk,layer(x=X1[1,:],y=X1[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))
# K1 = plotKDEContour(kde!(X1),xlbl="X (m)", ylbl="Y (m)",levels=4,layers=true);
# push!(plk,K1...)
# push!(plk,Gadfly.Theme(key_position = :none));

# L1 = getVal(getVariable(fg, beacon))
# L1 = rand(getVertKDE(fg,:l1),1000)
# push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
# K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=6,layers=true);
# push!(plk,K1...)
# push!(plk,Gadfly.Theme(key_position = :none));
# push!(plk, Guide.xlabel("X (m)"),Guide.ylabel("Y (m)"))
# push!(plk, Coord.cartesian(xmin=35, xmax=45, ymin=-50, ymax=-35,fixed=true))

# Check the range distribution
bss = AliasingScalarSampler(mfin[:,1,4],exp.(mfin[:,2,4]),SNRfloor=0.8)
out = rand(bss,1000)
Gadfly.plot(x=out,Geom.histogram,Coord.cartesian(xmin=0, xmax=100, ymin=0, ymax=400,fixed=true))




## dehann dev

using RoMEPlotting

# tree = wipeBuildNewTree!(fg, maxparallel=200)
# spyCliqMat(tree, :x169)

drawTree(tree, show=true)


plotTreeProductDown(fg, tree, :x169, :x169)
plotTreeProductDown(fg, tree, :x169, :x170)


#
getCliq(tree, :x169)


# fldr = "2019-09-01T12:25:09.297"
fldr = "2019-09-02T12:29:03.747"
# fg31_au = loadDFG("/tmp/caesar/"*fldr*"/cliqSubFgs/cliq31/fg_afterupsolve", Main)
# fg31_bu = loadDFG("/tmp/caesar/"*fldr*"/cliqSubFgs/cliq31/fg_beforeupsolve", Main)

fg31bd = initfg()
loadDFG("/tmp/caesar/"*fldr*"/cliqSubFgs/cliq31/fg_beforedownsolve", Main, fg31bd)
# fg31_bd =



writeGraphPdf(fg31bd, show=true)


plotLocalProduct(fg31bd, :x169)
plotLocalProduct(fg31bd, :x170)


printCliqHistorySummary(hist, tree, :x169)



sfg31bd = buildSubgraphFromLabels(fg31bd, [:x167;:x168;:x169;:x170])


writeGraphPdf(sfg31bd, show=true)


tree31s, smt31s, hist31s = solveTree!(sfg31bd)

drawTree(tree31s, show=true, imgs=true)


plotKDE(sfg31bd, ls(sfg31bd), levels=1)

plotLocalProduct(sfg31bd, :x169)
plotLocalProduct(sfg31bd, :x168)



# need a new factor plot debug function
#




plotVariableGivenFactor(sfg31bd,:x168x169f1,:x169)

pts = approxConv(sfg31bd,:x168x169f1, :x169)
lie = ls(sfg31bd, :x168x169f1)
setdiff!(lie, [:x169])


fct = getFactor(sfg31bd, :x168x169f1)
@show getFactorType(fct)
getData(fct).fncargvID

#
