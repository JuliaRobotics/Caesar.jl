

using Caesar, DelimitedFiles, JLD
using IncrementalInference
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

include(joinpath(@__DIR__,"slamUtils.jl"))

topdir = joinpath(ENV["HOME"],"data", "kayaks")

exptype = 1
if exptype == 1
    trialstr = "20_gps_pos";
    # frames = collect(1:1341);
    datawindow = collect(400:600);
elseif exptype == 2
    trialstr = "08_10_parsed";
    expnum = "/exp1";
    # frames = collect(1545:1950)
    datawindow = collect(1545:1645);
elseif exptype == 3
    trialstr = "07_18_parsed_set1";
    frames = collect();
end
datadir = joinpath(topdir,trialstr);
rangedir = joinpath(topdir, "rangeOnly_"*trialstr)

window = 1:datawindow[end]-datawindow[1]
mfin = zeros(7501,2,length(window));
posData = zeros(length(window),2);
for i in window
    dataindex = datawindow[i];
    navfile = datadir*"/nav$(dataindex).csv"
    posData[i,:] = readdlm(navfile,',',Float64,'\n')
    rangeFile = rangedir*"/range$(dataindex).txt";
    mfin[:,:,i] = readdlm(rangeFile,',',Float64,'\n')
end

dposData = deepcopy(posData)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

# mykde = kde!(rangesin[:,3]);
# K1 = plotKDE(mykde)

#Gadfly.plot(layer(x=posData[:,1],y=posData[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path))

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

rangewindow = window[1]:20:window[end]
for i in rangewindow
    sym = Symbol("x$i")
    ppR = Point2Point2Range(AliasingScalarSampler(mfin[:,1,i],exp.(mfin[:,2,i]),SNRfloor=0.8))
    addFactor!(fg, [sym;beacon], ppR, autoinit=false)
end

#writeGraphPdf(fg, engine="dot")
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = false

# tree, smt = batchSolve!(fg,maxparallel=100)

tree, smt, hist = solveTree!(fg, maxparallel=100)
# fg2 = deepcopy(fg)
# tree, smt, hist = solveTree!(fg,tree,maxparallel=100)

include(joinpath(@__DIR__,"plotSASUtils.jl"))


plotSASDefault(fg,posData,1,datadir=datadir)

#PLOTTING ------------
plk= [];
pkde = [];

if exptype ==1 #plot gt
    igt = [17.0499;1.7832];

    push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));
else
    ijldname = datadir * expnum * ".jld"
    iload = load(ijldname)
    igt = iload["icarus_gt"]
    windowstart = iload["ibegin"];
    windowend = iload["iend"];
    push!(plk,layer(x=igt[:,1],y=igt[:,2], label=String["Beacon Ground Truth";],Geom.path,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));
end

onetime = true;
plotbeacon = true;
for var in window       #Plot only for range factors
    sym = Symbol("x$var")
    global onetime
    for mysym in ls(fg,sym)
        if occursin(r"l1",string(mysym))
            # if var > 130 && !onetime       #Plot one or more approxConv
                L1ac = approxConv(fg,mysym,:l1)
                # K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=3,layers=true)
                # push!(plk,K1...)
                push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
                # push!(pkde,plotKDE(kde!(L1ac),layers=true)...)
                # onetime = true
                # push!(plk,Gadfly.Theme(key_position = :none));
                # push!(plk, Guide.xlabel("X (m)"), Guide.ylabel("Y (m)"))

                X1 = getKDEMean(getVertKDE(fg,sym))
                push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"magenta",point_size=1.5pt,highlight_width=0pt)))
            # end
            X1 = getKDEMean(getVertKDE(fg,sym))
            push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"red",point_size=1.5pt,highlight_width=0pt)))
        end
    end
end

#Ground Truth Trajectory
push!(plk, layer(x=posData[:,1],y=posData[:,2], Geom.path,Theme(default_color=colorant"green",point_size = 1.5pt,highlight_width = 0pt)))

#KDE Mean Vehicle Locations
for var in window
    sym = Symbol("x$var")
    X1 = getKDEMean(getVertKDE(fg,sym))
    push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"blue",point_size=1pt,highlight_width=0pt)))
end

if plotbeacon #Beacon Final solve
    L1v = getVariable(fg, beacon)
    L1 = getVal(L1v)
    push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
    K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
    push!(plk,K1...)
    push!(plk,Gadfly.Theme(key_position = :none));
end

push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-150, ymax=30,fixed=true))
# push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")

# pkdeplot = Gadfly.plot(pkde...); pkdeplot |> PDF("/tmp/test.pdf")

# Plot KDE for specific matched filtered range
# mykde = kde!(rangesin[:,260]);
# K1 = plotKDE(mykde); K1 |> PDF("/tmp/testkde.pdf")
# getKDEMax(mykde)
