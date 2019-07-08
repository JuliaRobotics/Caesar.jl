
using Caesar, DelimitedFiles
using KernelDensityEstimatePlotting
using Gadfly, Cairo, Fontconfig

# include(joinpath(@__DIR__,"slamUtils.jl"))
#
# rangesin = zeros(1000,320);
# for files in collect(60:20:360)
#     rangeFile = joinpath(ENV["HOME"],"data", "kayaks","rangesonly_6_20","range$(files).txt");
#     rangesin[:,files-60+1:files-60+20] = readdlm(rangeFile,',',Float64,'\n')
# end
# datadir = joinpath(ENV["HOME"],"data", "kayaks","20_gps_pos")
#
# window = 70:20:130;
#
# posDataAll = zeros(window[end]-window[1]+1,2);
# for i in window[1]:window[end]
#     navfile = datadir*"/nav$i.csv"
#     posDataAll[i-window[1]+1,:] = readdlm(navfile,',',Float64,'\n')
# end
#
# dposData = deepcopy(posDataAll)
# cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])
#
# #Gadfly.plot(layer(x=posDataAll[:,1],y=posDataAll[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path))
#
# fg = initfg();
# beacon = :l1
# addVariable!(fg, beacon, Point2 )
#
# fullwindow = 1:window[end]-window[1]+1
# fullwindowt = 1:window[end]-window[1]
# for i in fullwindow
#     sym = Symbol("x$i")
#     addVariable!(fg, sym, Point2)
# end

include(joinpath(@__DIR__,"slamUtils.jl"))

rangesin = zeros(1000,320);
for files in collect(60:20:360)
    rangeFile = joinpath(ENV["HOME"],"data", "kayaks","rangesonly_6_20","range$(files).txt");
    rangesin[:,files-60+1:files-60+20] = readdlm(rangeFile,',',Float64,'\n')
end
datadir = joinpath(ENV["HOME"],"data", "kayaks","20_gps_pos")

window = 70:20:130;
window = 70:20:100;

posDataAll = zeros(window[end]-window[1]+1,2);
for i in window[1]:window[end]
    navfile = datadir*"/nav$i.csv"
    posDataAll[i-window[1]+1,:] = readdlm(navfile,',',Float64,'\n')
end

dposData = deepcopy(posDataAll)
cumulativeDrift!(dposData,[0.0;0],[0.2,0.2])

#Gadfly.plot(layer(x=posDataAll[:,1],y=posDataAll[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path))

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
        addFactor!(fg, [sym;], pp, autoinit=false)

        dx = dposData[i+1,1] - posDataAll[i,1];
        dy = dposData[i+1,2] - posDataAll[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
    else
        dx = dposData[i+1,1] - dposData[i,1];
        dy = dposData[i+1,2] - dposData[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
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


# writeGraphPdf(fg, engine="dot")

for i in fullwindowt
    sym = Symbol("x$i")
    nextsymi = i+1;
    nextsym = Symbol("x$nextsymi")
    rtkCov = Matrix(Diagonal([0.1;0.1].^2));

    if i == fullwindow[1] || i == fullwindow[end]-1
        pp = PriorPoint2(MvNormal(posDataAll[i,:], rtkCov))
        addFactor!(fg, [sym;], pp, autoinit=false)

        dx = dposData[i+1,1] - posDataAll[i,1];
        dy = dposData[i+1,2] - posDataAll[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
        p2p2 = Point2Point2(MvNormal(dpμ,dpσ))
        addFactor!(fg, [sym;nextsym], p2p2, autoinit=false)
    else
        dx = dposData[i+1,1] - dposData[i,1];
        dy = dposData[i+1,2] - dposData[i,2];
        dpμ = [dx;dy];
        dpσ = Matrix(Diagonal([0.5;0.5].^2))
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


# writeGraphPdf(fg, engine="dot")

getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
getSolverParams(fg).async = true
getSolverParams(fg).downsolve = false


tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg)) # [:l1;]


L1v = getVariable(fg, beacon)
L1 = getVal(L1v)
plk = plotKDEContour(getVertKDE(fg,:l1),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
push!(plk,Gadfly.Theme(key_position = :none));

# f1 = getFactor(fg,:l1x60f1)
# pl12 = Gadfly.plot(x=L1[1,:],y=L1[2,:], Geom.histogram2d); pl12 |> PDF("/tmp/test.pdf")

for var in window
    windowi = var-window[1]+1
    sym = Symbol("x$windowi")
    X1 = getKDEMean(getVertKDE(fg,sym))
    push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.point))
    # X1 = getVal(getVariable(fg,sym))
    # push!(plk, layer(x=X1[1,:],y=X1[2,:], Geom.histogram2d))
    # navfile = datadir*"/nav$var.csv"
    # posData = readdlm(navfile,',',Float64,'\n')
    # push!(plk, layer(x=[posData[1];],y=[posData[2];], Geom.point,Theme(default_color=colorant"green")))
end

igt = [17.0499;1.7832];
push!(plk,layer(x=[igt[1];],y=[igt[2];], label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"red")));

plkplot = Gadfly.plot(plk...); plkplot |> PDF("/tmp/test.pdf")
@async run(`evince /tmp/test.pdf`)









## dev work below

getVariableType(fg, :x1)


lsfPriors(fg)
# lsfPriors(fg) = Symbol[:x1f2, :x1f1, :x60f1, :x60f2]

## moved to DistributedFactorGraphs
# """
#     $SIGNATURES
#
# Return `::Bool` on whether given factor `fc::Symbol` is a prior in factor graph `dfg`.
# """
# function isPrior(dfg::G, fc::Symbol)::Bool where G <: AbstractDFG
#   fco = getFactor(dfg, fc)
#   getfnctype(fco) isa FunctorSingleton
# end
#
# """
#     $SIGNATURES
#
# Return vector of prior factor symbol labels in factor graph `dfg`.
# """
# function lsfPriors(dfg::G)::Vector{Symbol} where G <: AbstractDFG
#   priors = Symbol[]
#   fcts = lsf(dfg)
#   for fc in fcts
#     if isPrior(dfg, fc)
#       push!(priors, fc)
#     end
#   end
#   return priors
# end
# """
# $SIGNATURES
#
# Return the DFGVariable softtype in factor graph `dfg<:AbstractDFG` and label `::Symbol`.
# """
# getVariableType(var::DFGVariable) = getSofttype(var)
# function getVariableType(dfg::G, lbl::Symbol) where G <: AbstractDFG
#     getVariableType(getVariable(dfg, lbl))
# end

getFactorType(fg, :x1f1)

function lsfTypes(dfg::G) where G <: AbstractDFG
  alltypes = String[]
  for fc in lsf(dfg)
    union!(alltypes, getFactorType(dfg, fc)
  end
end


#
