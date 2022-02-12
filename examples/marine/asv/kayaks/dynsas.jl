using Caesar, RoME, IncrementalInference
using RoMEPlotting, Gadfly, KernelDensityEstimatePlotting, Cairo, Fontconfig
using KernelDensityEstimate
using DocStringExtensions
using DelimitedFiles, JLD, DistributedFactorGraphs

include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))
include(joinpath(@__DIR__,"expDataUtils.jl"))

expID = "drift"
datastart = 1550
dataend = 1590
fgap = 10
gps_gap = 20
saswindow = 8
trialID = 1
debug = true;
dataloc = "local"

sas_gap = saswindow+fgap;
tstart = 1_000_000;
element = 2

totalposes = dataend-datastart;
datawindow = collect(datastart:dataend);
allpaths = getPaths(expID,"dynsas", currloc = dataloc, trialID = trialID);
scriptHeader = joinpath(allpaths[3],expID*"_fgap$(fgap)_gpsgap$(gps_gap)_swind$(saswindow)_f$(datastart)t$(dataend)_")

poses = Dict{Int,Array}();
nav = Dict{Int,Array}();
errorinds = [];
allsasframes = Dict{Int,Array}();

igt = loadGT(datawindow,expID,allpaths);
posData = loadNav(datawindow,allpaths)
dposData = deepcopy(posData)
i=1
while i+gps_gap < size(dposData,1)
    global i
    cumulativeDrift!(@view(dposData[i:i+gps_gap-1,:]),[0.0;0],[0.2,0.2])
     i+=gps_gap
    if i+gps_gap > size(dposData,1)
        cumulativeDrift!(@view(dposData[i-gps_gap:end,:]),[0.0;0],[0.2,0.2])
    end
end

# posPl = Gadfly.plot(layer(x=posData[:,1],y=posData[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path)); posPl |> PDF(scriptHeader*"posGT.pdf")

chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");
cfgFile = joinpath(ENV["HOME"],"data","sas","SAS2D.yaml")
cfgd=loadConfigFile(cfgFile)

fg = initfg();
getSolverParams(fg).maxincidence = 400
beacon = :l1
addVariable!(fg, beacon, Point2 )
tree = emptyBayesTree();

pose_counter = 1
sas_counter = 1
sas_gap_counter = 0;

while pose_counter < totalposes-2
    global pose_counter
    global sas_counter
    global sas_gap_counter
    global tstart
    current_symbol = Symbol("x$pose_counter")
    addVariable!(fg, current_symbol, DynPoint2(ut=tstart))
    println("Adding Variable x$pose_counter")
    sas_gap_counter += 1
    tstart += 1_000_000

    #Interpolate velocity from GPS ground truth every gps_gap poses
   if pose_counter == 1 || mod(pose_counter,gps_gap) == 0
       xdotp = posData[pose_counter+1,1] - posData[pose_counter,1];
       ydotp = posData[pose_counter+1,2] - posData[pose_counter,2];
        dpμ = [posData[pose_counter,1];posData[pose_counter,2];xdotp;ydotp];
        dpσ = Matrix(Diagonal([0.1;0.1;0.2;0.2].^2))

        pp = DynPoint2VelocityPrior(MvNormal(dpμ,dpσ))
        addFactor!(fg, [current_symbol;], pp, autoinit=true)
        # initManual!(fg,current_symbol,kde!(rand(MvNormal(dpμ,dpσ),100)))
   end

   #odo factors
   if pose_counter > 1
       xdotp = dposData[pose_counter,1] - dposData[pose_counter-1,1];
       ydotp = dposData[pose_counter,2] - dposData[pose_counter-1,2];
       xdotf = dposData[pose_counter+1,1] - dposData[pose_counter,1];
       ydotf = dposData[pose_counter+1,2] - dposData[pose_counter,2];
       # dpμ = [xdotp;ydotp;xdotf-xdotp;ydotf-ydotp];
       dpμ = [xdotp;ydotp;0;0];
       dpσ = Matrix(Diagonal([0.2;0.2;0.2;0.2].^2))
       vp = VelPoint2VelPoint2(MvNormal(dpμ,dpσ))
       addFactor!(fg, [Symbol("x$(pose_counter-1)");current_symbol], vp, autoinit=true)
       # initManual!(fg,current_symbol,kde!(rand(MvNormal(dpμ,dpσ),100)))
   end

   if sas_gap_counter >= sas_gap
       println("Trying to add SAS at pose: $pose_counter")
       navframes = collect(pose_counter-saswindow+1:1:pose_counter)
       navtemp = posData[navframes,:];
       navchecked, errorind = sanitycheck_nav(navtemp);
       push!(errorinds,errorind);

       if navchecked
           poses[sas_counter] = [Symbol("x$i") for i in pose_counter-saswindow+1:pose_counter]
           currentstart = datastart+pose_counter-saswindow
           println("Adding SAS at dataframe: $currentstart")
           sasframes = collect(currentstart:1:currentstart+saswindow-1);
           allsasframes[sas_counter] = sasframes;
           waveformData = importdata_waveforms(sasframes,element,datadir=allpaths[1]);
           sas2d = prepareSAS2DFactor(saswindow, waveformData, rangemodel=:Correlator,cfgd=cfgd, chirpFile=chirpFile)
           addFactor!(fg, [beacon;poses[sas_counter]], sas2d, autoinit=false)

           getSolverParams(fg).drawtree = false
           getSolverParams(fg).showtree = false
           getSolverParams(fg).limititers = 500

           lstpose = Symbol("x$(pose_counter)");
           tmpInit = approxConv(fg,Symbol("x$(pose_counter-1)x$(pose_counter)f1"),lstpose);
           XXkde = manikde!(getManifold(getVariable(fg,lstpose)), tmpInit);
           setValKDE!(fg,lstpose,XXkde);

           if sas_counter > 1
               tree = solveTree!(fg,tree)
           else
               tree = solveTree!(fg)
           end

           writeGraphPdf(fg,viewerapp="", engine="neato", filepath=scriptHeader*"fg.pdf")
           drawTree(tree, filepath=scriptHeader*"bt.pdf")

           plotSASDefault(fg,expID, posData,igt,dposData,datadir=allpaths[1],savedir=scriptHeader*"$sas_counter.pdf")

           if debug
               plk = [];
               push!(plk,plotBeaconGT(igt));
               # plotBeaconContours!(plk,fg);
               for mysym in poses[sas_counter]
                   xData = getKDEMax(getBelief(getVariable(fg,mysym)))
                   push!(plk, plotPoint(xData,colorIn=colorant"orange"))
               end
               L1pd = predictbelief(fg, :l1,ls(fg, :l1))
               L1ac = L1pd[1];

               L1 = getKDEMax(kde!(L1ac))
               push!(plk,layer(x=[L1[1];],y=[L1[2];], label=String["Beacon Max";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"darkorange",highlight_width = 0pt)));

               L1 = getKDEMean(kde!(L1ac))
               push!(plk,layer(x=[L1[1];],y=[L1[2];], label=String["Beacon Mean";],Geom.point,Geom.label(hide_overlaps=false), order=2, Theme(default_color=colorant"orangered",highlight_width = 0pt)))

               push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))

               K1 = plotKDEContour(kde!(L1ac),xlbl="X (m)", ylbl="Y (m)",levels=5,layers=true);
               push!(plk,K1...)
               push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
               Gadfly.plot(plk...) |> PDF(scriptHeader*"debug$sas_counter.pdf");
           end

           if expID == "dock"
               l1fit = getKDEMean(getBelief(getVariable(fg,:l1)))
               meanerror = sqrt((igt[1]-l1fit[1])^2+(igt[2]-l1fit[2])^2)

               l1max = getKDEMax(getBelief(getVariable(fg,:l1)))
               maxerror = sqrt((igt[1]-l1max[1])^2+(igt[2]-l1max[2])^2)

               mykde = getBelief(getVariable(fg,:l1))
               mynorm = kde!(rand(fit(MvNormal,getVal(fg,:l1)),200))
               kld = min(abs(KernelDensityEstimate.kld(mynorm,mykde)),abs(KernelDensityEstimate.kld(mykde,mynorm)))
           elseif expID == "drift"
               l1fit = getKDEMean(getBelief(getVariable(fg,:l1)))
               meanerror = sqrt((mean(igt[:,1])-l1fit[1])^2+(mean(igt[:,2])-l1fit[2])^2)

               l1max = getKDEMax(getBelief(getVariable(fg,:l1)))
               maxerror = sqrt((mean(igt[:,1])-l1max[1])^2+(mean(igt[:,2])-l1max[2])^2)

               mykde = getBelief(getVariable(fg,:l1))
               mynorm = kde!(rand(fit(MvNormal,getVal(fg,:l1)),200))
               kld = min(abs(KernelDensityEstimate.kld(mynorm,mykde)),abs(KernelDensityEstimate.kld(mykde,mynorm)))
           end

           jldname2 = scriptHeader * "solve_$(sas_counter).jld"
           JLD.save(jldname2,"beacon",getVal(fg,:l1),"posData",posData,"dposData", dposData,"gps_gap", gps_gap, "poses",poses,"sasframes", allsasframes, "l1fit",l1fit, "meanerror",meanerror,"l1max",l1max,"maxerror",maxerror,"kld",kld)

           # saveDFG(fg,scriptHeader * "fg$(sas_counter)")

           writedlm(scriptHeader*"stats.txt", [meanerror maxerror kld], ",")

           sas_counter +=1
           sas_gap_counter = 0
       end
   end
   pose_counter+=1

   println("Next Variable is $pose_counter")
   println("Next SAS is $sas_counter")
end
