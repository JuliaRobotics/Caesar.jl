

using Caesar, DelimitedFiles
using IncrementalInference
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

include(joinpath(@__DIR__,"slamUtils.jl"))

rangeswindow = 400:20:720
rangeskip = 20;
datadir = joinpath(ENV["HOME"],"liljondir", "kayaks","20_gps_pos")

# rangeswindow = 1510:40:1910
# rangeskip = 40;
# datadir = joinpath(ENV["HOME"],"liljondir", "kayaks","08_10_parsed")

rangesin = zeros(1000,rangeswindow[end]-rangeswindow[1]+rangeskip);
for files in rangeswindow
    rangeFile = joinpath(ENV["HOME"],"liljondir", "kayaks","rangesonly_6_20","range$(files).txt");
    # rangeFile = joinpath(ENV["HOME"], "data","sas","rangesonly_8_10_1","range$(files).txt");
    rangesin[:,files-rangeswindow[1]+1:files-rangeswindow[1]+rangeskip] = readdlm(rangeFile,',',Float64,'\n')
end

window = 420:20:700;
posDataAll = zeros(window[end]-window[1]+1,2);
for i in window[1]:window[end]
    navfile = datadir*"/nav$i.csv"
    posDataAll[i-window[1]+1,:] = readdlm(navfile,',',Float64,'\n')
end

dposData = deepcopy(posDataAll)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

# mykde = kde!(rangesin[:,3]);
# K1 = plotKDE(mykde)

#Gadfly.plot(layer(x=posDataAll[:,1],y=posDataAll[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path))

fg = initfg();
beacon = :l1
addVariable!(fg, beacon, Point2 )

fullwindow = 1:window[end]-window[1]+1
for i in fullwindow
    sym = Symbol("x$i")
    addVariable!(fg, sym, Point2)
    manualinit!(fg,sym,kde!(rand(MvNormal(dposData[i,:],Diagonal([0.5;0.5].^2)),100)))
end

for i in fullwindow
    sym = Symbol("x$i")
    nextsymi = i+1;
    nextsym = Symbol("x$nextsymi")
    rtkCov = Matrix(Diagonal([0.1;0.1].^2));

    if i == fullwindow[1]
        pp = PriorPoint2(MvNormal(posDataAll[i,:], rtkCov))
        addFactor!(fg, [sym;], pp, autoinit=false)

        dx = dposData[i+1,1] - dposData[i,1];
        dy = dposData[i+1,2] - dposData[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
    elseif  i == fullwindow[end]
        pp = PriorPoint2(MvNormal(posDataAll[i,:], rtkCov))
        addFactor!(fg, [sym;], pp, autoinit=false)
    else
        dx = dposData[i+1,1] - dposData[i,1];
        dy = dposData[i+1,2] - dposData[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
    end
end

for i in window
    windowi = i-window[1]+1
    rtkCov = Matrix(Diagonal([0.1;0.1].^2));
    sym = Symbol("x$windowi")
    mykde = kde!(rangesin[:,windowi])
    ppR = Point2Point2Range(mykde)
    addFactor!(fg, [sym;beacon], ppR, autoinit=false)
end

#writeGraphPdf(fg, engine="dot")
getSolverParams(fg).drawtree = false
getSolverParams(fg).showtree = false

# tree, smt = batchSolve!(fg,maxparallel=100)

tree, smt, hist = solveTree!(fg, maxparallel=100)
# fg2 = deepcopy(fg)
# tree, smt, hist = solveTree!(fg,tree,maxparallel=100)


#PLOTTING ------------
plk= [];
pkde = [];
igt = [17.0499;1.7832];

onetime = false;
for var in fullwindow       #Plot only for range factors
    sym = Symbol("x$var")
    global onetime
    for mysym in ls(fg,sym)
        if occursin(r"l1",string(mysym))
            # if var > 260 && !onetime       #Plot one or more approxConv
            #     L1ac = approxConv(fg,mysym,:l1)
            #     # K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=3,layers=true)
            #     # push!(plk,K1...)
            #     push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
            #     push!(pkde,plotKDE(kde!(L1ac),layers=true)...)
            #     onetime = true
            # end
            X1 = getKDEMean(getVertKDE(fg,sym))
            push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"red",point_size=1.5pt,highlight_width=0pt)))
        end
    end
end

#Ground Truth Trajectory
push!(plk, layer(x=posDataAll[:,1],y=posDataAll[:,2], Geom.path,Theme(default_color=colorant"green",point_size = 1.5pt,highlight_width = 0pt)))

#KDE Mean Vehicle Locations
for var in fullwindow
    sym = Symbol("x$var")
    X1 = getKDEMean(getVertKDE(fg,sym))
    push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point,Theme(default_color=colorant"blue",point_size=1pt,highlight_width=0pt)))
end

#Beacon Final solve
L1v = getVariable(fg, beacon)
L1 = getVal(L1v)
push!(plk,layer(x=L1[1,:],y=L1[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
K1 = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
push!(plk,K1...)
push!(plk,Gadfly.Theme(key_position = :none));

push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red",highlight_width = 0pt)));

push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-150, ymax=30))

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")

# pkdeplot = Gadfly.plot(pkde...); pkdeplot |> PDF("/tmp/test.pdf")

# Plot KDE for specific matched filtered range
# mykde = kde!(rangesin[:,260]);
# K1 = plotKDE(mykde); K1 |> PDF("/tmp/testkde.pdf")
# getKDEMax(mykde)
