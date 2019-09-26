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

# const TU = TransformUtils

Gadfly.set_default_plot_size(35cm,25cm)

include(joinpath(@__DIR__,"Plotting.jl"))
include(joinpath(@__DIR__,"SandsharkUtils.jl"))



# Step: Selecting a subset for processing and build up a cache of the factors.
epochs = timestamps[50:3:300]

lastepoch = 0
for ep in epochs
  global lastepoch
  if lastepoch != 0
    # @show interp_yaw(ep)
    deltaAng = interp_yaw(ep) - interp_yaw(lastepoch)

    wXi = TU.SE2([interp_x(lastepoch);interp_y(lastepoch);interp_yaw(lastepoch)])
    wXj = TU.SE2([interp_x(ep);interp_y(ep);interp_yaw(ep)])
    iDXj = se2vee(wXi\wXj)
    NAV[ep] = iDXj
    # println("$(iDXj[1]), $(iDXj[2]), $(iDXj[3])")

    odoDict[ep] = Pose2Pose2(MvNormal(NAV[ep], Matrix(Diagonal([0.1;0.1;0.005].^2))))
  end
  rangepts = rangedata[ep][:]
  rangeprob = kde!(rangepts)
  azipts = azidata[ep][:,1]
  aziprob = kde!(azipts)

  # alternative range probability
  rawmf = readdlm("/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/matchedfilter/raw/$(ep).csv",',')
  range_bss = AliasingScalarSampler(rawmf[:,1], exp.(rawmf[:,2]), SNRfloor=0.6)

  # prep the factor functions
  ppbrDict[ep] = Pose2Point2BearingRange(aziprob, range_bss) # rangeprob
  lastepoch = ep
end



function runEpochs!(fgl, STEP::Int, index::Vector{Int})
    for ep in epochs[(STEP+1):(STEP+10)]
      curvar = Symbol("x$(index[1])")
      addVariable!(fgl, curvar, Pose2)

      # xi -> l1 - nonparametric factor
      if index[1] % 5 == 0
          addFactor!(fgl, [curvar; :l1], ppbrDict[ep], autoinit=true)
      end

      if ep != epochs[1]
        # Odo factor x(i-1) -> xi
        addFactor!(fgl, [Symbol("x$(index[1]-1)"); curvar], odoDict[ep], autoinit=true)
      else
        # Prior to the first pose location (a "GPS" prior)
        initLoc = [interp_x(ep);interp_y(ep);interp_yaw(ep)]
        println("Adding a prior at $curvar, $initLoc")
        addFactor!(fgl, [curvar], PriorPose2( MvNormal(initLoc, Matrix(Diagonal([0.1;0.1;0.05].^2))) ), autoinit=true)
      end
      # Heading partial prior
      addFactor!(fgl, [curvar], RoME.PartialPriorYawPose2(Normal(interp_yaw(ep), deg2rad(3))), autoinit=true)
      index[1] += 1
    end
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
addFactor!(fg1, [:l1], PriorPose2( MvNormal([0.6; -16], Matrix(Diagonal([0.1; 0.1].^2)) ) ), autoinit=true)

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

    runEpochs!(fg1, STEP, index1)
    poses = sortVarNested(ls(fg1, r"x"))

    if STEP-10 >= 0
        @show poses[1], poses[end], poses[end-10]
        @show dellist = poses[1:(end-10-1)]
        deleteEpochs!(fg1, dellist)
        addFactor!(fg1, [poses[end-10];], PriorPose2(storeLast[poses[end-10]]))
    end

    saveDFG(fg1, joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_before.pdf"))
    @time tree1, smt, hist = solveTree!(fg1, tree1)
    saveDFG(fg1, joinpath(getSolverParams(fg1).logpath, "graphs", "fg_$(STEP)_after.pdf"))

    pla = drawPosesLandmarksAndOdo(fg1, ppbrDict, navkeys, X, Y, lblX, lblY)
    plb = plotSandsharkFromDFG(fg1)
    hstack(pla,plb) |> PDF(joinpath(getSolverParams(fg1).logpath, "sandshark-beacon_$STEP.pdf"))

    poses = sortVarNested(ls(fg1, r"x"))
    storeLast[poses[end]] = getKDE(fg1, poses[end])
    @show keys(storeLast)
end







## debugging a plotting


## plot forward convolves

fcts = setdiff(ls(fg1, :l1), lsfPriors(fg1))


pts = approxConv(fg1, fcts[1], :l1)

pts, infd = predictbelief(fg1,:l1,fcts)

pl1 = Gadfly.plot(x=pts[1,:],y=pts[2,:],Geom.hexbin)

union!(PL.layers, pl1.layers)

PL

plotKDE(manikde!(pts, Point2().manifolds))




dfg = fg1
fctsym = :x20l1f1
fct = getFactorType(dfg, fctsym)

#


function plotFactor(dfg::G, fctsym::Symbol, fct::Pose2Point2Bearing; hdl=[]) where G <: AbstractDFG

end


function plotFactor(dfg::G, fctsym::Sybmol, fct::Pose2Point2BearingRange) where G <: AbstractDFG

hdl = []

# variables
vars = ls(dfg, fctsym)

# the pose
pose = intersect(vars, ls(dfg, Pose2))[1]
poin = intersect(vars, ls(dfg, Point2))[1]

# plot current pose & point
pl_poin = plotKDE(dfg, poin, levels=5, c=["red";])
pl_pose = plotPose(dfg, pose, c=["black"])



# project landmark

# do range model separately too
pr = Pose2Point2Range(fct.range)

plotFactor(dfg, fctsym, pr, hdl=hdl)




hdl

end


plhist


## development

fct = Pose2Point2Bearing(fct.bearing)


pr = Pose2Point2Range(fct.range)

function plotFactor(dfg::G, fctsym::Symbol, fct::Pose2Point2Bearing) where {G <: AbstractDFG}

hdl = []

# variables
vars = ls(dfg, fctsym)

# the pose
pose = intersect(vars, ls(dfg, Pose2))[1]
poin = intersect(vars, ls(dfg, Point2))[1]

poseyaw = plotKDECircular(marginal(getKDE(dfg, pose), [3]), title="yaw angle")
bearpl = plotKDECircular(fct.bearing, title="bearing body frame")

vstack(poseyaw, bearpl)


end


function approxConv_debug(dfg::G) where G <: AbstractDFG



end


# """
#     $SIGNATURES
#
# Solve for the measurement values, leaving all variables in tact.
# """
# function inverseEvaluateFactor(dfg::G, fct::F) where {G <: AbstractDFG, F <: DFGFactor}
#
#
# end







## overview

PL = drawPosesLandmarksAndOdo(fg1, ppbrDict, navkeys, X, Y, lblX, lblY)


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




ROT = Float64[]


for (tim, data) in azidata
  global ROT
  ROT = [ROT; data[:,1]]
end




Gadfly.plot(x=ROT, Geom.histogram)


st = sort(collect(keys(azidata)))[150]

Gadfly.plot(x=azidata[st][:,1], Geom.histogram)



Gadfly.plot(x=getPoints(PX)[:], Geom.histogram)



mask = rand(Categorical(0.001*ones(1000)),100)
pts = collect(azidata[st][mask,1:1]')
PX = manikde!( pts, (:Circular,))
AMP.plotKDECircular([PX;], rVo=[10.0; 0.0; 0.0])



Gadfly.plot(
  layerBeamPatternRose( PX, wTRr= , wRr=wRr )
)








## Plot just dead reckoning


ep = epochs[1]
initLoc = [interp_x(ep);interp_y(ep);interp_yaw(ep)]


#



XX= Float64[]
YY = Float64[]
TH = Float64[]

PL = []

for i in 1:2:80
  @show ep = epochs[i]

  push!(XX, interp_x(ep))
  push!(YY, interp_y(ep))
  push!(TH, interp_yaw(ep)) # bad at wrap boundary

  mask = rand(Categorical(0.001*ones(1000)),100)
  pts = collect(azidata[ep][mask,1:1]')
  PX = manikde!( TU.wrapRad.(deepcopy(pts)), (:Circular,))
  # PX = manikde!( pts, (:Euclid,))

  pl = plotSandsharkFromDFG(dfg)
  # pll = AMP.plotKDECircular([PX;], rVo=[XX[end]; YY[end]; TH[end]], radix=1.2, scale=0.3)
  # push!(PL, pll)
  # his1 = Gadfly.plot(x=azidata[ep][:,1], Geom.histogram)
  # his2 = Gadfly.plot(x=pts, Geom.histogram)
  # vstack(his1, his2, pll) |> PDF("/tmp/caesar/random/debug/$ep.pdf")
end

pl = Gadfly.plot(
  Gadfly.layer(x=XX, y=YY, Geom.path),
  Gadfly.layer(x=[0.6;], y=[-16.0;], Geom.point, Theme(default_color=colorant"red")),
)

for pll in PL
  union!(pl.layers, pll.layers)
end

pl




## plot raw data in loop


i = 0
for t in timestamps
  @show t
  global i += 1
  pl1 = Gadfly.plot(x=azidata[t][:,1], y=azidata[t][:,2], Geom.point)
  pl2 = Gadfly.plot(x=)
   # |> PNG("/tmp/plots/$i.png")

end





## plot all beam patterns from factor graph

dfg = fg1

plotSandsharkFromDFG(dfg)

ls(dfg, Point2)
ls(dfg, Pose2)

ls(dfg, Pose2Pose2)





## RANGE MODELING

fct_range = getFactorType(fg1, :x45l1f1).range

plotKDE(fct_range)

rawmf = readdlm("/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/matchedfilter/raw/$(epochs[45]).csv",',')
bss = AliasingScalarSampler(rawmf[:,1], rawmf[:,2], SNRfloor=0.6)

pts = rand(bss,100)
Gadfly.plot(x=rawmf[:,1], y=rawmf[:,2], Geom.path)

Gadfly.plot(x=pts, Geom.histogram)

pts = rand(fct_range, 1000)






#
