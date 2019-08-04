
using Caesar, DelimitedFiles
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig


include(joinpath(@__DIR__,"slamUtils.jl"))

rangesin = zeros(1000,320);
for files in collect(60:20:360)
    rangeFile = joinpath(ENV["HOME"],"data", "kayaks","rangesonly_6_20","range$(files).txt");
    rangesin[:,files-60+1:files-60+20] = readdlm(rangeFile,',',Float64,'\n')
end
datadir = joinpath(ENV["HOME"],"data", "kayaks","20_gps_pos")

# window = 70:20:130;
window = 70:20:90;

posDataAll = zeros(window[end]-window[1]+1,2);
for i in window[1]:window[end]
    navfile = datadir*"/nav$i.csv"
    posDataAll[i-window[1]+1,:] = readdlm(navfile,',',Float64,'\n')
end

dposData = deepcopy(posDataAll)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])



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
        pp = PriorPoint2(MvNormal(posDataAll[i,:], rtkCov))
        addFactor!(fg, [sym;], pp, autoinit=true)

        dx = dposData[i+1,1] - posDataAll[i,1];
        dy = dposData[i+1,2] - posDataAll[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=true)
    else
        dx = dposData[i+1,1] - dposData[i,1];
        dy = dposData[i+1,2] - dposData[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=true)
    end
end

for i = window
    windowi = i-window[1]+1
    sym = Symbol("x$windowi")
    mykde = kde!(rangesin[:,i-window[1]+1])
    ppR = Point2Point2Range(mykde)
    addFactor!(fg, [beacon;sym], ppR, autoinit=false)
end



# writeGraphPdf(fg, engine="dot")
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
getSolverParams(fg).async = true
getSolverParams(fg).multiproc = false
getSolverParams(fg).downsolve = false



tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg))



include(joinpath(@__DIR__,"plotSASUtils.jl"))

# plotSASDefault(fg,posData,1,datadir=datadir)
plotSASDefault(fg,"test",posData,igt,dposData, datadir=allpaths[1], savedir="/tmp/caesar/test.pdf")
run(`evince /tmp/caesar/test.pdf`)


# getCliqInitUpMsgs(whichCliq(tree,:x6))


# smt47_it = @async Base.throwto(smt[47],InterruptException())


L1v = getVariable(fg, beacon)
L1 = getVal(L1v)
plk = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
push!(plk,Gadfly.Theme(key_position = :none));

# f1 = getFactor(fg,:l1x60f1)
# pl12 = Gadfly.plot(x=L1[1,:],y=L1[2,:], Geom.histogram2d); pl12 |> PDF("/tmp/test.pdf")

for var in window
    windowi = var-window[1]+1
    sym = Symbol("x$windowi")
    X1 = getKDEMean(getKDE(fg,sym))
    push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point))
end

igt = [17.0499;1.7832];
push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red")));

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")
@async run(`evince /tmp/test.pdf`)
# plkplot |> PNG("/tmp/test.png")





## other visualizations

using RoMEPlotting

vars = ls(fg, r"x")
svars = sortVarNested(vars)




## dev work below
