using Caesar, RoME, IncrementalInference
using RoMEPlotting, Gadfly, KernelDensityEstimatePlotting, Cairo, Fontconfig
using KernelDensityEstimate
using DocStringExtensions
using DelimitedFiles, JLD, DistributedFactorGraphs

include(joinpath(@__DIR__,"slamUtils.jl"))
include(joinpath(@__DIR__,"plotSASUtils.jl"))
include(joinpath(@__DIR__,"expDataUtils.jl"))

function main(expID::String, datastart::Int, dataend::Int, fgap::Int, gps_gap::Int; saswindow::Int=8, trialID::Int=1, debug::Bool=false, dataloc::String="lj")
    ## Default parameters
    println("Start Script\n")

    sas_gap = saswindow+fgap;
    tstart = 1_000_000;
    element = 2

    totalposes = dataend-datastart;
    datawindow = collect(datastart:dataend);
    allpaths = getPaths(expID,"dynsas", currloc = dataloc, trialID = trialID);
    scriptHeader = joinpath(allpaths[3],expID*"_fgap$(fgap)_gpsgap$(gps_gap)_swind$(saswindow)_f$(datastart)t$(dataend)_")
    savefgHeader = joinpath(allpaths[3]*"_fg",expID*"_fgap$(fgap)_gpsgap$(gps_gap)_swind$(saswindow)_f$(datastart)t$(dataend)_")

    poses = Dict{Int,Array}();
    nav = Dict{Int,Array}();
    errorinds = [];
    allsasframes = Dict{Int,Array}();

    igt = loadGT(datawindow,expID,allpaths);
    posData = loadNav(datawindow,allpaths)
    dposData = deepcopy(posData)
    i=1
    while i+gps_gap < size(dposData,1)
        cumulativeDrift!(@view(dposData[i:i+gps_gap-1,:]),[0.0;0],[0.2,0.2])
         i+=gps_gap
        if i+gps_gap > size(dposData,1)
            cumulativeDrift!(@view(dposData[i-gps_gap:end,:]),[0.0;0],[0.2,0.2])
        end
    end

    # posPl = Gadfly.plot(layer(x=posData[:,1],y=posData[:,2], Geom.path, Theme(default_color=colorant"green")), layer(x=dposData[:,1],y=dposData[:,2],Geom.path)); posPl |> PDF(scriptHeader*"posGT.pdf")

    chirpFile = joinpath("/media","data1","data","kayaks","chirp250.txt");
    cfgFile = joinpath("/media","data1","data","kayaks","SAS2D.yaml")
    cfgd=loadConfigFile(cfgFile)

    fg = initfg();
    beacon = :l1
    addVariable!(fg, beacon, Point2 )
    tree = emptyBayesTree();

    pose_counter = 1
    sas_counter = 1
    sas_gap_counter = 0;

    while pose_counter < totalposes-2
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
            # manualinit!(fg,current_symbol,kde!(rand(MvNormal(dpμ,dpσ),100)))
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
           # manualinit!(fg,current_symbol,kde!(rand(MvNormal(dpμ,dpσ),100)))
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
               XXkde = manikde!(tmpInit,getManifolds(fg,lstpose));
               setValKDE!(fg,lstpose,XXkde);

               if sas_counter > 1
                   tree, smt, hist = solveTree!(fg,tree, maxparallel=400)
               else
                   tree, smt, hist = solveTree!(fg, maxparallel=400)
               end

               writeGraphPdf(fg,viewerapp="", engine="neato", filepath=scriptHeader*"fg.pdf")
               drawTree(tree, filepath=scriptHeader*"bt.pdf")

               plotSASDefault(fg,expID, posData,igt,dposData,datadir=allpaths[1],savedir=scriptHeader*"$sas_counter.pdf")

               if debug
                   plk = [];
                   push!(plk,plotBeaconGT(igt));
                   plotBeaconContours!(plk,fg);
                   for mysym in poses[sas_counter]
                       xData = getKDEMax(getVertKDE(fg,mysym))
                       push!(plk, plotPoint(xData,colorIn=colorant"orange"))
                   end
                   L1p = approxConv(fg,ls(fg,:l1)[end],:l1);
                   push!(plk,layer(x=L1p[1,:],y=L1p[2,:],Geom.histogram2d(xbincount=300, ybincount=300)))
                   Gadfly.plot(plk...) |> PDF(scriptHeader*"debug$sas_counter.pdf");
               end

               if expID == "dock"
                   l1fit = getKDEMean(getVertKDE(fg,:l1))
                   meanerror = sqrt((igt[1]-l1fit[1])^2+(igt[2]-l1fit[2])^2)

                   l1max = getKDEMax(getVertKDE(fg,:l1))
                   maxerror = sqrt((igt[1]-l1max[1])^2+(igt[2]-l1max[2])^2)

                   mykde = getVertKDE(fg,:l1)
                   mynorm = kde!(rand(fit(MvNormal,getVal(fg,:l1)),200))
                   kld = min(abs(KernelDensityEstimate.kld(mynorm,mykde)),abs(KernelDensityEstimate.kld(mykde,mynorm)))
               elseif expID == "drift"
                   l1fit = getKDEMean(getVertKDE(fg,:l1))
                   meanerror = sqrt((mean(igt[:,1])-l1fit[1])^2+(mean(igt[:,2])-l1fit[2])^2)

                   l1max = getKDEMax(getVertKDE(fg,:l1))
                   maxerror = sqrt((mean(igt[:,1])-l1max[1])^2+(mean(igt[:,2])-l1max[2])^2)

                   mykde = getVertKDE(fg,:l1)
                   mynorm = kde!(rand(fit(MvNormal,getVal(fg,:l1)),200))
                   kld = min(abs(KernelDensityEstimate.kld(mynorm,mykde)),abs(KernelDensityEstimate.kld(mykde,mynorm)))
               end

               ev = 0;
               evi = 0;
               es = 0;
               for tmpi = 1:pose_counter
                   rv = getVal(fg,Symbol("x$tmpi"));
                   dxt = (rv[1,:].-posData[tmpi,1]).^2;
                   dyt = (rv[2,:].-posData[tmpi,2]).^2;
                   ev += sum(sqrt.(dxt+dyt));
                   dxt2 = (dposData[tmpi,1].-posData[tmpi,1]).^2;
                   dyt2 = (dposData[tmpi,2].-posData[tmpi,2]).^2;
                   evi += sum(sqrt.(dxt2+dyt2));

                   if tmpi > 1
                       dv = (rv[3,:].-(posData[tmpi,1]-posData[tmpi-1,1])).^2;
                       dw = (rv[4,:].-(posData[tmpi,2]-posData[tmpi-1,2])).^2;
                       es += sum(sqrt.(dv+dw));
                   end
               end
               ev = ev./pose_counter;
               evi = evi./pose_counter;
               es = es./(pose_counter-1);

               jldname2 = scriptHeader * "solve_$(sas_counter).jld"
               JLD.save(jldname2,"beacon",getVal(fg,:l1),"posData",posData,"dposData", dposData,"gps_gap", gps_gap, "poses",poses,"sasframes", allsasframes, "l1fit",l1fit, "meanerror",meanerror,"l1max",l1max,"maxerror",maxerror,"kld",kld,"ev",ev,"evi",ev, "es", es)

               saveDFG(fg,savefgHeader * "fg$(sas_counter)")

               writedlm(scriptHeader*"stats$(sas_counter).txt", [pose_counter meanerror maxerror kld evi ev es], ",")

               sas_counter +=1
               sas_gap_counter = 0
           end
       end
       pose_counter+=1

       println("Next Variable is $pose_counter")
       println("Next SAS is $sas_counter")

   end

   println("This Solve was saved under: "*scriptHeader)
   return fg
end
