

using Caesar, DelimitedFiles
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

include(joinpath(@__DIR__,"slamUtils.jl"))

rangesin = zeros(1000,320);
for files in collect(60:20:360)
    rangeFile = joinpath(ENV["HOME"],"liljondir", "kayaks","rangesonly_6_20","range$(files).txt");
    rangesin[:,files-60+1:files-60+20] = readdlm(rangeFile,',',Float64,'\n')
end
datadir = joinpath(ENV["HOME"],"liljondir", "kayaks","20_gps_pos")

window = 70:40:320;

posDataAll = zeros(window[end]-window[1]+1,2);
for i in window[1]:window[end]
    navfile = datadir*"/nav$i.csv"
    posDataAll[i-window[1]+1,:] = readdlm(navfile,',',Float64,'\n')
end

dposData = deepcopy(posDataAll)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

# Gadfly.plot(layer(x=posDataAll[:,1],y=posDataAll[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path))

fg = initfg();
beacon = :l1
addVariable!(fg, beacon, Point2 )

fullwindow = 1:window[end]-window[1]+1
fullwindowt = 1:window[end]-window[1]
for i in fullwindow
    sym = Symbol("x$i")
    addVariable!(fg, sym, Point2)
end

for i in fullwindowt
    sym = Symbol("x$i")
    nextsymi = i+1;
    nextsym = Symbol("x$nextsymi")
    rtkCov = Matrix(Diagonal([0.1;0.1].^2));

    if i == fullwindow[1] || i == fullwindow[end]-1
        # navfile = datadir*"/nav$i.csv"
        # posData = readdlm(navfile,',',Float64,'\n')
        pp = PriorPoint2(MvNormal(posDataAll[i,:], rtkCov))
        addFactor!(fg, [sym;], pp, autoinit=false)

        dx = dposData[i+1,1] - posDataAll[i,1];
        dy = dposData[i+1,2] - posDataAll[i,2];
        dpμ = [xdotp;ydotp];
        dpσ = Matrix(Diagonal([0.1;0.1].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
    else
        dx = dposData[i+1,1] - dposData[i,1];
        dy = dposData[i+1,2] - dposData[i,2];
        dpμ = [xdotp;ydotp];
        dpσ = Matrix(Diagonal([0.1;0.1].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
    end
end

for i = window
    windowi = i-window[1]+1
    sym = Symbol("x$windowi")
    mykde = kde!(rangesin[:,i-window[1]+1])
    ppR = Point2Point2Range(mykde)
    addFactor!(fg, [beacon;sym], ppR, autoinit=false)
end


writeGraphPdf(fg, engine="dot")

tree, smt, hist = solveTree!(fg, recordcliqs=[:l1;])

L1v = getVariable(fg, beacon)
L1 = getVal(L1v)
plk = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
push!(plk,Gadfly.Theme(key_position = :none));

# f1 = getFactor(fg,:l1x60f1)
# pl12 = Gadfly.plot(x=L1[1,:],y=L1[2,:], Geom.histogram2d); pl12 |> PDF("/tmp/test.pdf")

for var in window
    X1 = getVal(getVariable(fg, Symbol("x$var")))
    push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.histogram2d))
    navfile = datadir*"/nav$var.csv"
    posData = readdlm(navfile,',',Float64,'\n')
    push!(plk, layer(x=[posData[1];],y=[posData[2];], Geom.point))
end

igt = [17.0499;1.7832];
push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red")));

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")
