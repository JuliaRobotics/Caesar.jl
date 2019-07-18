# using Distributed
# addprocs(5)

using Caesar
using RoMEPlotting, KernelDensityEstimatePlotting
using KernelDensityEstimate, IncrementalInference
using DocStringExtensions, DelimitedFiles
using Gadfly, Cairo, Fontconfig

include(joinpath(@__DIR__,"slamUtils.jl"))

wstart = 415;
wlen = 9;

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
beacon = :l1
addVariable!(fg, beacon, Point2 )

poses = [Symbol("x$i") for i in 1:wlen]
for sym in poses
  addVariable!(fg, sym, Point2)
end

priors = [1,8]
rtkCov = Matrix(Diagonal([0.1;0.1].^2))
#Priors
for i in priors
    pp = PriorPoint2(MvNormal(posData[i,:], rtkCov))
    addFactor!(fg, [Symbol("x$i");], pp, autoinit=false)
end

vps = 1:wlen-1
dpσ = Matrix(Diagonal([0.2;0.2].^2))
for i in vps
    dx = dposData[i+1,1] - dposData[i,1];
    dy = dposData[i+1,2] - dposData[i,2];
    p2p2 = Point2Point2(MvNormal([dx;dy],dpσ))
    addFactor!(fg, [Symbol("x$i");Symbol("x$(i+1)")], p2p2, autoinit=false)
end

rangewindow = 1:3:wlen
for i in rangewindow
    sym = Symbol("x$i")
    ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
    addFactor!(fg, [sym;beacon], ppR, autoinit=false)
end

writeGraphPdf(fg, engine="neato")
wipeBuildNewTree!(fg, drawpdf=true, show=true)

getSolverParams(fg).drawtree = true
#getSolverParams(fg).showtree = true

## solve the factor graph
tree, smt, hist = solveTree!(fg, recordcliqs=[:x1; :l1; :x8])



# Debugging here

bss = AliasingScalarSampler(mfin[:,1,4],exp.(mfin[:,2,4]),SNRfloor=0.8)
out = rand(bss,1000)
Gadfly.plot(x=out,Geom.histogram)



getFrontals(tree.cliques[3])
cdict = getCliqChildMsgsUp(tree,tree.cliques[3],BallTreeDensity)

mykde = cdict[:x7][1][1];
plotKDE(marginal(mykde,[1;]),axis=[0.0 100],N=100)

plotKDE(getVertKDE(fg,:x4))
plk= [];

for sym in poses #plotting all syms labeled
    X1 = getKDEMean(getVertKDE(fg,sym))
    push!(plk, layer(x=[X1[1];],y=[X1[2];], label=["$(sym)";], Geom.point, Geom.label), Theme(default_color=colorant"red",point_size = 1.5pt,highlight_width = 0pt))
end

igt = [17.0499;1.7832];

push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));

L1 = getVal(getVariable(fg, beacon))
K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
push!(plk,K1...)
push!(plk,Gadfly.Theme(key_position = :none));
push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-150, ymax=75,fixed=true))

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf");



Profile.init(n = 10^7, delay = 0.01)
@profile stuff = sandboxCliqResolveStep(tree,:x3,6)
ProfileView.view()
Profile.print(format=:flat)

## Plot the SAS factor pairs
fsym = :l1x1x2x3x4x5f1
pl = plotSASPair(fg, fsym, show=true, filepath="/tmp/testDP_2pp_3vpvp_init.pdf");

plk=[]
X1 = getKDEMean(getVertKDE(fg,:x1))
push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point))
tplt = Gadfly.plot(plk...); tplt |> SVG("/tmp/test.svg") ; tplt |> PDF("/tmp/test.pdf")


plotKDE(fg, ls(fg,r"x"), dims=[1;2])


## More debugging below

stuff = IncrementalInference.localproduct(fg, :x1)
pl = plotKDE(stuff[1], dims=[1;2], levels=3, c=["blue"])
