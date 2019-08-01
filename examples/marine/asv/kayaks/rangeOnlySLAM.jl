using Caesar, DelimitedFiles, JLD
using IncrementalInference
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"expDataUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))

# expID = "dock"; datawindow = collect(400:700);
expID = "drift"; datawindow = collect(1550:1750);

allpaths = getPaths(expID,"range", trialID = 1);
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

rangewindow = window[1]:40:window[end]
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
getSolverParams(fg).downsolve = false


tree, smt, hist = solveTree!(fg, maxparallel=100)
drawTree(tree,filepath = "/tmp/test.pdf")
# tree, smt = batchSolve!(fg,maxparallel=100)
# fg2 = deepcopy(fg)
# tree, smt, hist = solveTree!(fg,tree,maxparallel=100)

plotSASDefault(fg,expID, posData,igt,datadir=allpaths[1], savedir="/tmp/caesar/test.pdf")
run(`evince /tmp/caesar/test.pdf`)


#PLOTTING ------------
plk= [];
pkde = [];

if exptype ==1 #plot gt
    igt = [17.0499;1.7832];

    push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));

    lkde = getVertKDE(fg, beacon)
    L1 = rand(lkde,1000);
    L1err = L1;
    L1err = sqrt.((L1[1,:] .- igt[1]).^2 + (L1[2,:] .- igt[2]).^2);
    totalerr = sum(L1err)/length(L1err);

else
    push!(plk,layer(x=igt[:,1],y=igt[:,2], label=String["Beacon Ground Truth";],Geom.path,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));
end

onetime = false;
plotbeacon = true;
for var in window       #Plot only for range factors
    sym = Symbol("x$var")
    global onetime
    for mysym in ls(fg,sym)
        if occursin(r"l1",string(mysym))
            if var > 130 && onetime       #Plot one or more approxConv
                L1ac = approxConv(fg,mysym,:l1)
                # K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=3,layers=true)
                # push!(plk,K1...)
                push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
                # push!(pkde,plotKDE(kde!(L1ac),layers=true)...)
                onetime = false
                # push!(plk,Gadfly.Theme(key_position = :none));
                # push!(plk, Guide.xlabel("X (m)"), Guide.ylabel("Y (m)"))

                X1 = getKDEMean(getVertKDE(fg,sym))
                push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"magenta",point_size=1.5pt,highlight_width=0pt)))
            end
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
    # L1v = getVariable(fg, beacon)
    # lkde = getVertKDE(fg, beacon)
    # L1 = rand(lkde,1000);
    # push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
    K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
    push!(plk,K1...)
    push!(plk,Gadfly.Theme(key_position = :none));
end

# push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-150, ymax=30,fixed=true))
push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=-25,fixed=true))

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")

# pkdeplot = Gadfly.plot(pkde...); pkdeplot |> PDF("/tmp/test.pdf")

# Plot KDE for specific matched filtered range
# mykde = kde!(rangesin[:,260]);
# K1 = plotKDE(mykde); K1 |> PDF("/tmp/testkde.pdf")
# getKDEMax(mykde)
