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


odonoise = Matrix(Diagonal((10*[0.1;0.1;0.005]).^2))



epochs, odoDict, ppbrDict, NAV = doEpochs(timestamps, rangedata, azidata, interp_x, interp_y, interp_yaw, odonoise, TEND=600)



# Build interpolators for x, y from LBL data
itpl_lblx = LinearInterpolation(lblkeys, lblX)
itpl_lbly = LinearInterpolation(lblkeys, lblY)

# quick look at initial location
itpl_lblx[epochs[1]]
itpl_lbly[epochs[1]]
interp_yaw[epochs[1]]

0

function runEpochs!(fgl, epochs, STEP::Int, index::Vector{Int}; acousticRate=3)

    metadata = open( joinpath(getSolverParams(fg1).logpath, "metadata.txt"), "a")

    for ep in epochs[(STEP+1):(STEP+10)]
      curvar = Symbol("x$(index[1])")
      addVariable!(fgl, curvar, Pose2)

      # xi -> l1 - nonparametric factor
      if index[1] % acousticRate == 0
          addFactor!(fgl, [curvar; :l1], ppbrDict[ep], autoinit=true)
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
      # addFactor!(fgl, [curvar], RoME.PartialPriorYawPose2(Normal(interp_yaw(ep), deg2rad(3))), autoinit=true)

      # write out some metadata to help with debugging
      println(metadata, "$(index[1]), $ep, $curvar")

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



## Step: Building the factor graph
fg1 = initfg()
# Add a central beacon with a prior
addVariable!(fg1, :l1, Point2)
# Pinger location is (0.6; -16)
addFactor!(fg1, [:l1], PriorPose2( MvNormal([17; 1.8], Matrix(Diagonal([0.1; 0.1].^2)) ) ), autoinit=true) # [0.6; -16]

# init tree for simpler code later down
tree1 = wipeBuildNewTree!(fg1)

## and some settings

# enable fixed lag operation
# getSolverParams(fg1).isfixedlag = true
# getSolverParams(fg1).qfl = 20

# first solve and initialization
getSolverParams(fg1).drawtree = true
# getSolverParams(fg).showtree = false

mkpath(joinpath(getSolverParams(fg1).logpath, "graphs"))


index1 = Int[0;]
index2 = Int[0;]
storeLast = Dict{Symbol,BallTreeDensity}()

for STEP in 0:10:20
    global fg1, tree1
    global index1, index2
    global storeLast

    runEpochs!(fg1, epochs, STEP, index1)
    poses = sortVarNested(ls(fg1, r"x"))

    if STEP-10 >= 0
        @show poses[1], poses[end], poses[end-10]
        @show dellist = poses[1:(end-10-1)]
        deleteEpochs!(fg1, dellist)
        addFactor!(fg1, [poses[end-10];], PriorPose2(storeLast[poses[end-10]]))
    end

    saveDFG(fg1, joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_before"))
    @time tree1, smt, hist = solveTree!(fg1, tree1)
    saveDFG(fg1, joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_after"))

    pla = drawPosesLandmarksAndOdo(fg1, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)
    plb = plotSandsharkFromDFG(fg1)
    hstack(pla,plb) |> PDF(joinpath(getSolverParams(fg1).logpath, "sandshark-beacon_$STEP.pdf"))

    reportFactors(fg1, Pose2Point2BearingRange, filepath=joinpath(getSolverParams(fg1).logpath, "acoustics_$STEP.pdf"), show=false)
    reportFactors(fg1, Pose2Pose2, filepath=joinpath(getSolverParams(fg1).logpath, "odo_$STEP.pdf"), show=false)

    poses = sortVarNested(ls(fg1, r"x"))
    storeLast[poses[end]] = getKDE(fg1, poses[end])
    @show keys(storeLast)
end



# sum(timestamps .< 1531153363000000000) # 348



plotLocalProduct(fg1, :x27, show=true)



plotKDE(ppbrDict[epochs[1]].bearing)





## debugging a plotting


dfg = fg1





drawGraph(dfg)


plotLocalProduct(dfg, :l1)
plotLocalProduct(dfg, :x100)

getSolverParams(dfg).dbg = true

tree, smt, hist = solveTree!(dfg, recordcliqs=ls(dfg))


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
