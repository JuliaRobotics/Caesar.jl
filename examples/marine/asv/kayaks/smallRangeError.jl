
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

poses = [Symbol("x$i") for i in 1:wlen]
for sym in poses
  addVariable!(fg, sym, Point2)
end

priors = [1,8]
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
# initManual!(fg,beacon,kde!(rand(MvNormal([0;0],Matrix(Diagonal([7.0;7].^2))),100)))

rangewindow = 1:3:wlen
for i in rangewindow
    sym = Symbol("x$i")
    ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
    addFactor!(fg, [sym;beacon], ppR, autoinit=false)
end

writeGraphPdf(fg, engine="neato")
wipeBuildNewTree!(fg, drawpdf=true, show=true, imgs=false)
# getSolverParams(fg).drawtree = true
#getSolverParams(fg).showtree = true

## solve the factor graph , recordcliqs=[:x1; :l1; :x8]
tree = solveTree!(fg)

# using FunctionalStateMachine
# using IncrementalInference
# csmAnimate(fg,tree,[:x1; :l1; :x8])

# Debugging here

bss = AliasingScalarSampler(mfin[:,1,4],exp.(mfin[:,2,4]),SNRfloor=0.8)
out = rand(bss,1000)
Gadfly.plot(x=out,Geom.histogram,Coord.cartesian(xmin=0, xmax=100, ymin=0, ymax=400,fixed=true))


pdict = getCliqParentMsgDown(tree,tree.cliques[5])
mykde = pdict[:x4][1];


xx = rand(mykde,1000)
plt = Gadfly.plot(layer(x=xx,Geom.histogram(density=true)),Guide.xlabel("X,Y(m)"));  plt |> PDF("/tmp/plt.pdf");


plt = plotKDEContour(mykde,xlbl="X (m)", ylbl="Y (m)",levels=4); plt |> PDF("/tmp/plt.pdf")

getFrontals(tree.cliques[4])
cdict = getCliqChildMsgsUp(tree,tree.cliques[4],BallTreeDensity)

mykde = cdict[:x4][1][1];
# X1 = rand(mykde,400)
# plk = Gadfly.plot(layer(x=X1[1,:],y=X1[2,:],Geom.histogram2d(xbincount=400, ybincount=400)),Theme(key_position = :none));  plk |> PDF("/tmp/plt.pdf");

# plt = plotKDE(marginal(mykde,[1;]),axis=[0.0 100],N=100); plt |> PDF("/tmp/plt.pdf");

xx = rand(marginal(mykde,[2]),1000)
plt = Gadfly.plot(layer(x=xx,Geom.histogram(density=true)));  plt |> PDF("/tmp/plt.pdf");

plt = plotKDE(mykde,levels=4); plt |> "/tmp/plt.pdf"

plk= [];

for sym in poses #plotting all syms labeled
    X1 = getKDEMean(getBelief(fg,sym))
    push!(plk, layer(x=[X1[1];],y=[X1[2];], label=["$(sym)";], Geom.point, Geom.label, Theme(default_color=colorant"blue",point_size = 1.5pt,highlight_width = 0pt)))
    K1 = plotKDEContour(getBelief(fg,sym),xlbl="", ylbl="",levels=2,layers=true);
    push!(plk,K1...)
    push!(plk,Gadfly.Theme(key_position = :none));
end
push!(plk, Coord.cartesian(xmin=20, xmax=60, ymin=-60, ymax=-30,fixed=true))
plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf");

#
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
# L1 = rand(getBelief(fg,:l1),1000)
# push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
# K1 = plotKDEContour(getBelief(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=6,layers=true);
# push!(plk,K1...)
# push!(plk,Gadfly.Theme(key_position = :none));
# push!(plk, Guide.xlabel("X (m)"),Guide.ylabel("Y (m)"))
# push!(plk, Coord.cartesian(xmin=35, xmax=45, ymin=-50, ymax=-35,fixed=true))



Profile.init(n = 10^7, delay = 0.01)
@profile stuff = sandboxCliqResolveStep(tree,:x3,6)
ProfileView.view()
Profile.print(format=:flat)

## Plot the SAS factor pairs
fsym = :l1x1x2x3x4x5f1
pl = plotSASPair(fg, fsym, show=true, filepath="/tmp/testDP_2pp_3vpvp_init.pdf");

plk=[]
X1 = getKDEMean(getBelief(fg,:x1))
push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point))
tplt = Gadfly.plot(plk...); tplt |> SVG("/tmp/test.svg") ; tplt |> PDF("/tmp/test.pdf")


plotKDE(fg, ls(fg,r"x"), dims=[1;2])


## More debugging below

stuff = IncrementalInference.localproduct(fg, :x1)
pl = plotKDE(stuff[1], dims=[1;2], levels=3, c=["blue"])





## plotting for dehann
# plk= [];
#
# for sym in poses #plotting all syms labeled
#     X1 = getKDEMean(getBelief(fg,sym))
#     push!(plk, layer(x=[X1[1];],y=[X1[2];], label=["$(sym)";], Geom.point, Geom.label, Theme(default_color=colorant"blue",point_size = 1.5pt,highlight_width = 0pt)))
#     K1 = plotKDEContour(getBelief(fg,sym),xlbl="", ylbl="",levels=2,layers=true);
#     push!(plk,K1...)
#     push!(plk,Gadfly.Theme(key_position = :none));
# end
# push!(plk, Coord.cartesian(xmin=20, xmax=50, ymin=-60, ymax=-30,fixed=true))
# plkplot = Gadfly.plot(plk...)
