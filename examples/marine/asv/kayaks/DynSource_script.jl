using Caesar, RoME, IncrementalInference
using RoMEPlotting, Gadfly, KernelDensityEstimatePlotting
using KernelDensityEstimate
using DocStringExtensions
using DelimitedFiles, JLD

include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))

expnum = 2;
if expnum == 1
    topdir = joinpath(ENV["HOME"],"data","kayaks","07_18")
    trialstr = "07_18_parsed_set1";
    datadir = joinpath(topdir,trialstr);
    ijldname = datadir * "/exp1" * ".jld"
elseif expnum==2
    topdir = joinpath(ENV["HOME"],"data","kayaks","07_18")
    trialstr = "07_18_parsed_set2";
    datadir = joinpath(topdir,trialstr);
    ijldname = datadir * "/exp1" * ".jld"
end

cfgFile = joinpath(ENV["HOME"],"data","sas","SAS2D.yaml");
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");
cfgd=loadConfigFile(cfgFile)
cfgd["range_snr_floor"]=0.9
iload = load(ijldname)
nrange = iload["nrange"]
irange = iload["irange"];
itemp = readdlm(datadir*"/inav.csv",',',Float64,'\n', skipstart=1);

wstart = 190; wlen = 25;
dataframes = collect(nrange[wstart]:nrange[wstart]+wlen);

saswindow = 5;
sasstart = 1;
nsasfac = 5;
sasdataframes = [zeros(Int,saswindow) for _ in 1:nsasfac];
sasposes = [];
irangenew = irange[wstart] : 5 : irange[wstart+nsasfac]
igt = [itemp[irangenew,2] itemp[irangenew,4]];

poses = [Symbol("x$i") for i in 1:wlen]
posData = importdata_nav(dataframes,datadir=datadir);
navchecked, errorind = sanitycheck_nav(posData)

# dposData = deepcopy(posData)
# cumulativeDrift!(dposData,[0.0;0],[0.1,0.1])

# Check the trajectory
posPl = Gadfly.plot(layer(x=posData[:,1],y=posData[:,2], Geom.point, Geom.path, Theme(default_color=colorant"green",highlight_width = 0pt)),layer(x=igt[:,1],y=igt[:,2], Geom.point, Geom.path, Theme(default_color=colorant"red" ,highlight_width = 0pt))); posPl |> PDF("/tmp/test.pdf");

for i=1:nsasfac
    framestart = nrange[wstart]+sasstart-1+(i-1)*saswindow;
    sasdataframes[i] = collect(framestart:framestart+saswindow-1)
    push!(sasposes,[Symbol("x$j") for j in sasstart+(i-1)*saswindow:sasstart+saswindow-1+(i-1)*saswindow])
end

fg = initfg();
rtkCov= Matrix(Diagonal([0.1;0.1].^2));
for i in 1:wlen
  addVariable!(fg, poses[i], Point2)
  pp = PriorPoint2(MvNormal(posData[i,:], rtkCov))
  addFactor!(fg, [poses[i];], pp, autoinit=true)
end

beacons = [Symbol("l$j") for j in 1:nsasfac]

for i=1:nsasfac
    addVariable!(fg, beacons[i], DynPoint2(ut=1_000_000+(i-1)*saswindow) )

    if i>1
        dpμ = [0.0;0;0;0];
        dpσ = Matrix(Diagonal([1.0;1;0.1;0.1].^2))
        vp = VelPoint2VelPoint2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [beacons[i-1];beacons[i]], vp, autoinit=false)
    end

    waveformData = importdata_waveforms(sasdataframes[i],2, datadir=datadir);
    sas2d = prepareSAS2DFactor(saswindow, waveformData, rangemodel=:Correlator,
                               cfgd=cfgd, chirpFile=chirpFile)
    addFactor!(fg, [beacons[i];sasposes[i]], sas2d, autoinit=false)
    @show currfac = ls(fg,beacons[i])[1]

    tmpInit = approxConv(fg,currfac,beacons[i]);
    XXkde = manikde!(getManifold(getVariable(fg,beacons[i])), tmpInit);
    setValKDE!(fg,beacons[i],XXkde);
end

# visualization tools for debugging
# writeGraphPdf(fg,viewerapp="", engine="neato", filepath = "/tmp/test.pdf")

# getSolverParams(fg).drawtree = true
getSolverParams(fg).limititers=500
#getSolverParams(fg).showtree = true

## solve the factor graph
tree = solveTree!(fg)

# Some Debuggging
# @time L1 = approxConv(fg,:l1x1x2x3x4x5x6x7x8f1,:l1)
# @time L1 = approxConv(fg,:l2x9x10x11x12x13x14x15x16f1,:l2)

# plk = plotBeaconSolve(fg)
plk=[];
for sym in ls(fg) #plotting all syms labeled
    X1 = getKDEMean(getBelief(getVariable(fg,sym)))
    push!(plk, layer(x=[X1[1];],y=[X1[2];],label=String["$sym";], Geom.point,Geom.label), Theme(default_color=colorant"red",point_size = 1.5pt,highlight_width = 0pt))
end
# push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))

K1 = plotKDEContour(kde!(getVal(fg,:l1)[1:2,:]),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
push!(plk,K1...)
push!(plk,Gadfly.Theme(key_position = :none));

push!(plk,layer(x=posData[:,1],y=posData[:,2], Geom.point, Geom.path, Theme(default_color=colorant"blue")), layer(x=igt[:,1],y=igt[:,2], Geom.point, Geom.path, Theme(default_color=colorant"green" )))
push!(plk, Coord.cartesian(xmin=0, xmax=100, ymin=-50, ymax=20,fixed=true))
plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")


plk= [];


push!(plk,layer(x=posData[:,1],y=posData[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=igt[:,1],y=igt[:,2], Geom.point, Geom.path, Theme(default_color=colorant"red" )))

L1 = getVal(getVariable(fg, :l1))
push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))

# push!(plk, Coord.cartesian(xmin=40, xmax=100, ymin=-120, ymax=-50,fixed=true))
plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf");


## Plot the SAS factor pairs
fsym = :l1x1x2x3x4x5x6x7x8f1
pl = plotSASPair(fg, fsym, show=true, filepath="/tmp/testDP_2pp_3vpvp_init.pdf");
