addprocs(5)

using RoME, Distributions
using KernelDensityEstimate
using IncrementalInference
using TransformUtils, Colors

using RoMEPlotting, Gadfly
using KernelDensityEstimatePlotting
using JLD2

# for development
importall Caesar

include(joinpath(dirname(@__FILE__),"slamUtils.jl"))
Gadfly.push_theme(:default)

saswindow = 8
N = 100

beacon = :l1
# addFactor!(fg, [:l1], Prior(MvNormal(zeros(2), diagm([100;100]) )) )

symbolstart = 1;
factorgap = 5;

windowstart = 55;
nfactors = 40;
posegap = 2;

poses = Dict{Int,Array}();
nav = Dict{Int,Array}();
errorinds = [];
allframes = Dict{Int,Array}();
saveBF = Dict();

#datadir = "/media/data1/data/kayaks/07_18_parsed/";
savedirheader = "0813/0813";

fg = initfg()
addNode!(fg, beacon, Point2)

for fitr in 1:nfactors
    navchecked = false;
    while ~navchecked
        sasframes = collect(windowstart:1:(windowstart+saswindow-1))
        navtemp = importdata_nav(sasframes);
        navchecked, errorind = sanitycheck_nav(navtemp);
        push!(errorinds,errorind);
        if navchecked
            nav[fitr] = navtemp;
            poses[fitr] = [Symbol("x$i") for i in symbolstart:(symbolstart+saswindow-1)]
            addsascluster_only_gps!(fg, poses[fitr], beacon, sasframes, N=N)
            allframes[fitr] = sasframes
        else
            windowstart+=errorind
        end
    end

    if !isInitialized(fg,:l1)
        IncrementalInference.doautoinit!(fg, :l1)
    end

    jldname2 = "$(pwd())/" * savedirheader * "_nav_$(fitr).jld"
    save(jldname2,"nav",nav,"poses",poses,"frames",allframes)

    tree = wipeBuildNewTree!(fg, drawpdf=false)
    inferOverTree!(fg, tree, N=N)

    PL = [];

    beacongt = [17.0499;1.7832];
    push!(PL,layer(x=[beacongt[1];],y=[beacongt[2];],label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2));

    l1fit = getKDEMean(getVertKDE(fg,:l1))
    push!(PL,layer(x=[l1fit[1];],y=[l1fit[2];],label=String["KDE Mean";],Geom.point,Geom.label(hide_overlaps=false),order=1));

    l1max = getKDEMax(getVertKDE(fg,:l1))
    push!(PL,layer(x=[l1max[1];],y=[l1max[2];],label=String["KDE Max";],Geom.point,Geom.label(hide_overlaps=false),order=1));

    push!(PL, plotKDE(getVertKDE(fg, :l1),levels=4, layers=true)... )
    L1 = getVal(fg, :l1)
    push!(PL, layer(x=L1[1,:],y=L1[2,:], Geom.histogram2d))

    for i in 1:fitr
        push!(PL,layer(x=nav[i][:,1],y=nav[i][:,2],Geom.point, Geom.path));
        saveBF[(fitr,i)] = approxConvFwdBFRaw(fg, poses[i], :l1, nav[i][1,:])
    end

    push!(PL,Coord.Cartesian(xmin=-100,xmax=150,ymin=-150,ymax=100))
    pl = plot(PL...);

    plname = savedirheader * "_$(fitr).png"
    Gadfly.draw(PNG(plname,1.5*20cm,1.5*20cm),pl)

    jldname = savedirheader * "_fg_$(fitr).jld"
    savejld(fg,file=jldname)

    symbolstart += (saswindow + factorgap)
    windowstart += (saswindow + posegap)
end


#poses1 = [Symbol("x$i") for i in 1:(1+saswindow-1)]
#windowstart = 100;
#sasframes = collect(windowstart:1:windowstart+saswindow-1)
#nav1 = importdata_nav(sasframes);
#addsascluster_only_gps!(fg, poses1, beacon, sasframes, N=N)

# writeGraphPdf(fg)
# @async run(`evince bt.pdf`)


#PLOTTING TOOLS

Gadfly.push_theme(:default)

PL = [];

beacongt = [17.0499;1.7832];
push!(PL,layer(x=[beacongt[1];],y=[beacongt[2];],label=String["Beacon Ground Truth";],Geom.point,Geom.label(hide_overlaps=false), order=2));

l1fit = fit(MvNormal,getVal(fg,:l1))
push!(PL,layer(x=[l1fit.μ[1];],y=[l1fit.μ[2];],label=String["Mean-Fit";],Geom.point,Geom.label(hide_overlaps=false),order=1));

l1max = getKDEMax(getVertKDE(fg,:l1))
push!(PL,layer(x=[l1max[1];],y=[l1max[2];],label=String["Max-Fit";],Geom.point,Geom.label(hide_overlaps=false),order=1));

push!(PL, plotKDE(getVertKDE(fg, :l1),levels=4, layers=true)... )
#L1 = getVal(fg, :l1)
#push!(PL, layer(x=L1[1,:],y=L1[2,:], Geom.histogram2d))

saveBF = Dict()
for i in 1:nfactors
    #push!(PL,layer(x=nav[i][:,1],y=nav[i][:,2],Geom.point, Geom.path, Theme(default_color=colorant"red")));
    #push!(PL, plotKDE(getVertKDE(fg, Symbol("x$(i)")),levels=4, layers=true, dimLbls=["",""])... )
    saveBF[i] = approxConvFwdBFRaw(fg, poses[i], :l1, nav[i][1,:])
end

jldnameBF = "$(pwd())/0809/0809_BF_all.jld"
save(jldnameBF,"saveBF",saveBF)

push!(PL,Coord.Cartesian(xmin=150,xmax=250,ymin=-200,ymax=-50))

pl = plot(PL...)

frames = collect(15:425);
dataOut = zeros(length(frames),2);
count = 0
for i in frames
    count+=1
    navfile = datadir*"/nav$(i).csv"
    dataOut[count,:] = readdlm(navfile,',',Float64,'\n')
end

ll = ls(fg,:l1)

stuff = IncrementalInference.localProduct(fg,:l1)

allv = collect(1:12)
lo =[4,7,9,10,11]
# keep = [1;2;3;5;6;8;12]
keep = setdiff(allv, lo)
plotKDE([getVertKDE(fg,:l1);stuff[2][keep]],levels=1,c=["red";["blue" for i in 1:12]])


something = predictbelief(fg, :l1, ll[keep])


plotKDE([getVertKDE(fg,:l1);kde!(something);stuff[2][keep]],levels=1,c=["red";"cyan";["blue" for i in 1:12]])


plotKDE([getVertKDE(fg,:l1);kde!(something);stuff[2][keep]],levels=1,c=["red";"cyan";["blue" for i in 1:12]], dims=[1])


navfile = datadir*"inav.csv"
inav = readdlm(navfile,',',Float64,'\n')

push!(PL,layer(x=dataOut[:,1],y=dataOut[:,2],Geom.point, Geom.path,Theme(default_color=colorant"green")),
     layer(x=inav[:,1],y=inav[:,2]))

Gadfly.draw(SVG("stored/07-27-530-2.svg",1.5*20cm,1.5*20cm),pl)
savejld(fg,file="stored/07-27-530.jld")



## PREVIOUS DEVELOPMENT AND TESTING CODE BELOW

# backwards solve

# backwards leave one out case
setVal!(fg, :l1, 0.01*randn(2,N))
reset!(sas2d.dbg)
predX2 = IncrementalInference.approxConv(fg, :l1x1x2x3x4x5x6x7x8x9f1, :x9, N=N)





pl = plot(
layer(x=predX2[1,:], y=predX2[2,:], Geom.histogram2d(ybincount=200)),
layer(x=posData[1:9,1],y=posData[1:9,2], Geom.point)
)

pl.coord = Coord.Cartesian(xmin=0,xmax=20,ymin=-55,ymax=-35)
pl








## VISUALIZATION AND BEAMS DEBUGGIN




pl = plotKDE(kde!(predL1))
# pl.coord = Gadfly.Coord.Cartesian(xmin=-200, xmax=100, ymin=-100, ymax=200);
pl




# Look at sas2d.dbg
# @show length(sas2d.dbg)
plot(x=sas2d.dbg.azi_smpls, Geom.histogram)


n=200; pl = plot(x=linspace(0,2pi,length(sas2d.dbg.beams[n])), y=sas2d.dbg.beams[n], Geom.path())


n=200; pl = plot(x=linspace(0,2pi,length(sas2d.dbg.beams_cut[n])), y=sas2d.dbg.beams_cut[n], Geom.path())




pl = plot(x=linspace(0,2pi,length(avg_beam)), y=avg_beam, Geom.path(), Coord.Cartesian(ymin=0))
pl = plot(x=linspace(0,2pi,length(avg_beam_cut)), y=avg_beam_cut, Geom.path(), Coord.Cartesian(ymin=0))








#
