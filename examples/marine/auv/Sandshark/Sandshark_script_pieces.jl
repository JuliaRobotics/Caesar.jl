# new Sandshark example
# add more julia processes
using Distributed
addprocs(4)

using Caesar, RoME
@everywhere using Caesar, RoME
using Interpolations
using Distributions

using RoMEPlotting, ApproxManifoldProducts
using Gadfly, Fontconfig, Cairo
using DataFrames
using ProgressMeter
using DelimitedFiles


@everywhere setForceEvalDirect!(true)



Gadfly.set_default_plot_size(35cm,25cm)

include(joinpath(@__DIR__,"Plotting.jl"))
include(joinpath(@__DIR__,"SandsharkUtils.jl"))


odonoise = Matrix(Diagonal((20*[0.1;0.1;0.005]).^2))



epochs, odoDict, ppbrDict, ppbDict, pprDict, NAV = doEpochs(timestamps, rangedata, azidata, interp_x, interp_y, interp_yaw, odonoise, TEND=800)

# Gadfly.plot(y=pprDict[epochs[1]].Z.weights.values, Geom.path) # Gadfly.plot(x=rand(pprDict[epochs[1]].Z, 100), Geom.histogram)
# using StatsBase
# fieldnames(StatsBase.ProbabilityWeights)

# Build interpolators for x, y from LBL data
itpl_lblx = LinearInterpolation(lblkeys, lblX)
itpl_lbly = LinearInterpolation(lblkeys, lblY)

# quick look at initial location
itpl_lblx[epochs[1]]
itpl_lbly[epochs[1]]
interp_yaw[epochs[1]]


DeltaX = itpl_lblx[epochs[1]] - interp_x[epochs[1]]
DeltaY = itpl_lbly[epochs[1]] - interp_y[epochs[1]]


0

global anicounter = 0

function runEpochs!(fgl, epochs, STEP::Int, index::Vector{Int}; acousticRate=3)
    global anicounter
    metadata = open( joinpath(getSolverParams(fg1).logpath, "metadata.txt"), "a")

    curvar = Symbol("x$(index[1])")
    # # point where real-time estimate pivots from SLAM to deadreckoning
    # if STEP > 0
    #   pivotPoseEpoch = epochs[STEP]
    #   @show pivotPoseSym = Symbol("x$(index[1]-1)")
    #   @show pivotPoseEst = calcVariablePPE(fgl, pivotPoseEst).suggested
    # end

    for ep in epochs[(STEP+1):(STEP+10)]
      addVariable!(fgl, curvar, Pose2)

      # xi -> l1 - nonparametric factor
      if index[1] % acousticRate == 0
          # addFactor!(fgl, [curvar; :l1], ppbrDict[ep], autoinit=true)
          addFactor!(fgl, [curvar; :l1], pprDict[ep], autoinit=false)
          # addFactor!(fgl, [curvar; :l1], ppbDict[ep], autoinit=false)
      end

      if ep != epochs[1]
        # Odo factor x(i-1) -> xi
        addFactor!(fgl, [Symbol("x$(index[1]-1)"); curvar], odoDict[ep], autoinit=true)
      else
        # Prior to the first pose location (a "GPS" prior)
        # initLoc = [interp_x(ep);interp_y(ep);interp_yaw(ep)]
        initLoc = [itpl_lblx(ep);itpl_lbly(ep);interp_yaw(ep)]
        println("Adding a prior at $curvar, $initLoc")
        addFactor!(fgl, [curvar], PriorPose2( MvNormal(initLoc, Matrix(Diagonal([0.1;0.1;0.05].^2))) ), autoinit=true)
      end
      # Heading partial prior
      addFactor!(fgl, [curvar], RoME.PartialPriorYawPose2(Normal(interp_yaw(ep), deg2rad(5))), autoinit=true)

      # write out some metadata to help with debugging
      println(metadata, "$(index[1]), $ep, $curvar")

      ensureAllInitialized!(fgl)

      # visualizations
      plre = drawPoses(fgl,drawhist=false,contour=false,spscale=1.0, lbls=false, meanmax=:mean)
      plre.coord = Coord.Cartesian(xmin=0, xmax=100, ymin=-120, ymax=-70)
      # add confidence contour on last
      pcf = plotKDE(getKDE(fgl, curvar),dims=[1;2],levels=1,c=["gray70"] )
      union!(plre.layers, pcf.layers)
      # export
      plre |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "fg_$curvar.png") )

      tfg = initfg()
      addVariable!(tfg, :l1, Point2)
      initManual!(tfg,:l1,getKDE(fgl,:l1))
      addVariable!(tfg, curvar, Pose2)
      initManual!(tfg,curvar,getKDE(fgl,curvar))
      addFactor!(tfg, [curvar;:l1], pprDict[ep])

      hdl = []
      plotFactor(tfg, ls(tfg, :l1)[1], hdl=hdl)
      hdl[7].coord = Coord.Cartesian(xmin=-150.0, xmax=200.0, ymin=-250.0, ymax=100.0)
      hdl[7] |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "range_$curvar.png") )

      # circular illustration
      cpts = reshape(rand(Normal(interp_yaw(ep), deg2rad(5)),100),1,:)
      cbm = manikde!(cpts, Sphere1)

      opts = index[1] > 0 ? approxConv(fgl, intersect(ls(fgl, Pose2Pose2),ls(fgl, curvar))[1], curvar) : 2*randn(3,100)
      cbo = manikde!(opts[3:3,:], Sphere1)
      plcb = plotKDECircular([cbm; cbo], legend=["mag";"odo"], c=["orange";"blue"])
      plcb.coord = Coord.Cartesian(xmin=-2.5, xmax=2.5, ymin=-2.5, ymax=2.5)
      plcb |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "circ_$curvar.png") )

      # animate dead reckoning
      if ep != epochs[1]
        aniT = LinearInterpolation([0.0;1.0], [epochs[index[1]];epochs[index[1]+1]]) # ep
        for atim in 0:0.1:1
          @show atim, aniT(atim)
          mask = epochs[1] .< navkeys .< aniT(atim)
          if sum(mask) > 2
            pldo = Gadfly.plot(x=[X[mask];interp_x[aniT(atim)]].+DeltaX, y=[Y[mask];interp_y[aniT(atim)]].+DeltaY, Geom.path(), Theme(default_color=colorant"gray80"))
            pldo |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "animation", "deadreck_$(aniT(atim)).png") )
            union!(pldo.layers, plre.layers)
            pldo.coord = Coord.Cartesian(xmin=0, xmax=100, ymin=-120, ymax=-70)
            hstack(pldo, vstack(plcb, hdl[7])) |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "animation",   "combined_$(anicounter).png") )
            anicounter += 1
          end
        end
      end

      # # animate the estimated location
      # if ep != epochs[1]
      #   aniT = LinearInterpolation([0.0;1.0], [epochs[index[1]];epochs[index[1]+1]]) # ep
      #   for atim in 0:0.1:1
      #       @show atim, aniT(atim)
      #       rtmask = pivotPoseEpoch .< navkeys .< aniT(atim)
      #       if sum(mask) > 2
      #           # get newest deadreckoning segment
      #           xRT=[X[rtmask];interp_x[aniT(atim)]].-interp_x[pivotPoseEpoch].+pivotPoseEst[1]) yRT=[Y[rtmask];interp_y[aniT(atim)]].-interp_y[pivotPoseEpoch].+pivotPoseEst[2])
      #           # plot pivot deadreckoning
      #           pldo = Gadfly.plot(x=xRT, y=yRT, Geom.path(), Theme(default_color=colorant"red"))
      #           pldo |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "animation", "rt_segment_$(aniT(atim)).png") )
      #       end
      #   end
      # end

      # combined plot
      hstack(plre, vstack(plcb, hdl[7])) |> PNG( joinpath(getSolverParams(fgl).logpath, "reckoning", "combined_$curvar.png") )

      index[1] += 1
    end
    close(metadata)

    nothing
end



function deleteEpochs!(fgl, lbls::Vector{Symbol})
  function delFactsVar!(fgl, lb::Symbol)
    fcts = ls(fgl, lb)
    map(x->deleteFactor!(fgl, x), fcts)
    deleteVariable!(fgl, lb)
    nothing
  end
  map(x->delFactsVar!(fgl, x), lbls)
  nothing
end




global anicounter = 0

## Step: Building the factor graph
fg1 = initfg()
# Add a central beacon with a prior
addVariable!(fg1, :l1, Point2)

# Pinger location is [17; 1.8]
beaconprior = PriorPoint2( MvNormal([17; 1.8], Matrix(Diagonal([0.1; 0.1].^2)) ) )
addFactor!(fg1, [:l1], beaconprior, autoinit=true)

# init tree for simpler code later down
tree1 = wipeBuildNewTree!(fg1)

## and some settings

# enable fixed lag operation
# getSolverParams(fg1).isfixedlag = true
# getSolverParams(fg1).qfl = 20

getSolverParams(fg1).maxincidence = 200

# first solve and initialization
getSolverParams(fg1).drawtree = true
# getSolverParams(fg).showtree = false

mkpath(joinpath(getSolverParams(fg1).logpath, "graphs"))
mkpath(joinpath(getSolverParams(fg1).logpath, "reckoning"))
mkpath(joinpath(getSolverParams(fg1).logpath, "reckoning", "animation"))


index1 = Int[0;]
index2 = Int[0;]
storeLast = Dict{Symbol,BallTreeDensity}()

laglength = 20

length(epochs)

for STEP in 0:10:50
    global fg1, tree1
    global index1, index2
    global storeLast

    runEpochs!(fg1, epochs, STEP, index1, acousticRate=3)
    poses = sortDFG(ls(fg1, r"x"))

    if STEP-laglength >= 0
        @show poses[1], poses[end], poses[end-laglength]
        @show dellist = poses[1:(end-laglength-1)]
        if length(dellist) > 0
          deleteEpochs!(fg1, dellist)
          addFactor!(fg1, [poses[end-laglength];], PriorPose2(storeLast[poses[end-laglength]]))
        end
    end

    drawGraph(fg1, filepath=joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_.pdf"))
    saveDFG(fg1, joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_before"))
    @time tree1, smt, hist = solveTree!(fg1, tree1)
    saveDFG(fg1, joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_after"))

    pla = drawPosesLandmarksAndOdo(fg1, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)
    plb = plotSandsharkFromDFG(fg1)
    hstack(pla,plb) |> PDF(joinpath(getSolverParams(fg1).logpath, "sandshark-beacon_$STEP.pdf"))

    reportFactors(fg1, Pose2Point2Range, filepath=joinpath(getSolverParams(fg1).logpath, "acoustics_$STEP.pdf"), show=false)
    reportFactors(fg1, Pose2Pose2, filepath=joinpath(getSolverParams(fg1).logpath, "odo_$STEP.pdf"), show=false)

    poses = sortVarNested(ls(fg1, r"x"))
    storeLast[poses[end]] = getKDE(fg1, poses[end])
    @show keys(storeLast)
end



##### END OF RUN, ANALYSIS BELOW================================================



## debug weird results before IIF v0.8.0

ls(fg1)
drawGraph(fg1)

getFactorType(fg1, ls(fg1, :x0)[2])


drawTree(tree1, show=true)
plotLocalProduct(fg1, :l1)
plotTreeProductDown(fg1,tree1,:l1)


plotLocalProduct(fg1, :x0)


solveTree!(fg1)

getFactorType(fg1, ls(fg1, :l1)[2])

reportFactors(fg1, Pose2Point2Bearing)

## debug weird



fcts = ls(fg1, :l1)
PTS = map(x->approxConv(fg1, x, :l1), fcts)

PP = map(x->manikde!(x, Point2), PTS)

pp = *(PP...)

plotKDE([pp; PP...], levels=2)


# sum(timestamps .< 1531153363000000000) # 348

ls(fg1, :l1)

##
fctsym = :x150l1f1

reportFactors(fg1, Pose2Point2Range, ls(fg1, Pose2Point2Range) )
reportFactors(fg1, Pose2Pose2, ls(fg1, Pose2Pose2) )

##

plotLocalProduct(fg1, :x27, show=true)



plotKDE(ppbrDict[epochs[1]].bearing)





## debugging a plotting


dfg = fg1





drawGraph(dfg)


plotLocalProduct(dfg, :l1)
plotLocalProduct(dfg, :x100)

getSolverParams(dfg).dbg = true

tree = solveTree!(dfg, recordcliqs=ls(dfg))


plotSandsharkFromDFG(dfg)
drawPosesLandmarksAndOdo(dfg, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)

# fg1_ = deepcopy(fg1)



0







## compare raw beam former data with factors


plotRawBeamFormer(fg1, ppbrDict, NAV, interp_x, interp_y, interp_yaw, show=true, all=true)



## load previous factor graph to compare
_fg = initfg()
loadDFG(joinpath(getSolverParams(fg1).logpath, "graphs", "fg_0_after.pdf"), Main, _fg)



drawGraph(_fg)

plotPose(_fg, :x0)
getFactorType)





## report on factors


dfg = fg1

fctsym = ls(dfg, Pose2Point2BearingRange)[1]
# fcss = lsf(dfg, Pose2Pose2)


plotFactor(dfg, :x20x21f1)
plotFactor(dfg, :x20l1f1)


reportFactors(dfg, Pose2Pose2, ls(dfg, Pose2Pose2)[1:2])
reportFactors(dfg, Pose2Point2BearingRange)






## overview

PL = drawPosesLandmarksAndOdo(fg1, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)


plotSandsharkFromDFG(fg1, scale=1.5)





## slightly random

drawGraph(fg1)
plotLocalProduct(fg1, :x55)
sortVarNested(ls(fg1, r"x"))
pp, parr, partials, lb, infdim = IIF.localProduct(fg1, :x55)
plotKDE([pp;parr], levels=1, legend=["prod"; string.(lb[1:3])], dims=[1;2])
plotPose(fg1, :x59)






## DEBUG beam pattern timestamps

epochs

pll = layerBeamPatternRose(ppbrDict[ep].bearing, wRr=wRr, wTRr=theta[1:2], scale=5.0)







#




## plot all beam patterns from factor graph


plotSandsharkFromDFG(dfg)





## RANGE MODELING

fct_range = getFactorType(fg1, :x45l1f1).range

plotKDE(fct_range)

rawmf = readdlm("/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/matchedfilter/raw/$(epochs[45]).csv",',')
bss = AliasingScalarSampler(rawmf[:,1], rawmf[:,2], SNRfloor=0.6)

pts = rand(bss,100)
Gadfly.plot(x=rawmf[:,1], y=rawmf[:,2], Geom.path)

Gadfly.plot(x=pts, Geom.histogram)

pts = rand(fct_range, 1000)






## plot forward convolves

fcts = setdiff(ls(fg1, :l1), lsfPriors(fg1))


pts = approxConv(fg1, fcts[1], :l1)

pts, infd = predictbelief(fg1,:l1,fcts)

pl1 = Gadfly.plot(x=pts[1,:],y=pts[2,:],Geom.hexbin)

union!(PL.layers, pl1.layers)

PL

plotKDE(manikde!(pts, Point2().manifolds))





#








# pts = collect(azidata[1531153769000000000][:,1:1]')
#
# ptsw = TU.wrapRad.(pts)
#
# pc = manikde!(pts, Sphere1)
# pl = manikde!(pts, ContinuousScalar)
# plw = manikde!(ptsw, ContinuousScalar)
#
# plotKDE([pl;plw])
