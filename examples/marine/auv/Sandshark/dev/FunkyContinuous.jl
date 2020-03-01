# new Sandshark example
# add more julia processes
# nprocs() < 7 ? addprocs(7-nprocs()) : nothing

using KernelDensityEstimate, IncrementalInference
using Distributions

const TU = TransformUtils

## Step: Building the factor graph
fg = initfg()
# Add a central beacon with a prior
addVariable!(fg, :l1, ContinuousScalar)
# Pinger location is (0.6; -16)
addFactor!(fg, [:l1], IIF.Prior( Normal(0.6, 0.1) ))

epochs = 50:2:100

index=0
for ep in epochs
    curvar = Symbol("x$index")
    addVariable!(fg, curvar, ContinuousScalar)

    if ep != epochs[1]
      # Odo factor x(i-1) -> xi
      addFactor!(fg, [Symbol("x$(index-1)"); curvar], LinearConditional(Normal(1, 0.1)))
    else
      # Prior to the first pose location (a "GPS" prior)
      addFactor!(fg, [curvar], IIF.Prior(Normal(5,5) ))
    end
    # Heading partial prior
    # addFactor!(fg, [curvar], RoME.PartialPriorYawPose2(Normal(interp_yaw(ep), deg2rad(3))))
    index+=1
end

# Just adding the first one...
addFactor!(fg, [:x0; :l1], LinearConditional(Normal(1.0, 0.1)))

addFactor!(fg, [:x5; :l1], LinearConditional(Normal(1.0, 0.1)))

addFactor!(fg, [:x10; :l1], LinearConditional(Normal(1.0, 0.1)))

addFactor!(fg, [:x13; :l1], LinearConditional(Normal(1.0, 0.1)))

addFactor!(fg, [:x15; :l1], LinearConditional(Normal(1.0, 0.1)))

addFactor!(fg, [:x17; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x18; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x19; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x20; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x21; :l1], LinearConditional(Normal(1.0, 0.1))) # breaks it!
addFactor!(fg, [:x22; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x23; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x24; :l1], LinearConditional(Normal(1.0, 0.1)))
addFactor!(fg, [:x25; :l1], LinearConditional(Normal(1.0, 0.1)))

plotPose(fg, [:x21])
plotKDE(fg, [:x20, :x21, :x22, :x23, :x24])
# plotKDE(ppbrDict[epochs[26]].range)
#
# plotKDE(ppbrDict[epochs[23]].bearing)
plotKDE(ppbrDict[epochs[23]].range)

writeGraphPdf(fg, engine="dot")

ensureAllInitialized!(fg)
solveTree!(fg)

drawPosesLandms(fg)

# saveDFG(fg, file="presolve_$t.jld")
IIF.solveTree!(fg) #, N=100
# saveDFG(fg, file="postsolve_$t.jld")

# pl = drawPoses(fg, spscale=2.75) # Just for odo plot
# Roll again for inspiration check
## PLOT BEAM PATTERNS
Gadfly.push_theme(:default)
pla = drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)
Gadfly.draw(PDF("sandshark-beacon_$t.pdf", 12cm, 15cm), pla)
Gadfly.draw(PNG("sandshark-beacon_$t.png", 12cm, 15cm), pla)




## resolve funky 21


# shows all the proposals based on the clique in the tree -- similar to plotLocalProduct on factor graph
plotTreeProduct(fg, tree, :x25)
#
# using IncrementalInference
#
#
# import RoMEPlotting: spyCliqMat
#
# function spyCliqMat(cliq::Graphs.ExVertex; showmsg=true)
#   mat = deepcopy(IIF.getCliqMat(cliq, showmsg=showmsg))
#   # TODO -- add improved visualization here, iter vs skip
#   mat = map(Float64, mat)*2.0-1.0
#   numlcl = size(IIF.getCliqAssocMat(cliq),1)
#   mat[(numlcl+1):end,:] *= 0.9
#   mat[(numlcl+1):end,:] -= 0.1
#   numfrtl1 = floor(Int,length(cliq.attributes["data"].frontalIDs)+1)
#   mat[:,numfrtl1:end] *= 0.9
#   mat[:,numfrtl1:end] -= 0.1
#   @show cliq.attributes["data"].itervarIDs
#   @show cliq.attributes["data"].directvarIDs
#   @show cliq.attributes["data"].msgskipIDs
#   @show cliq.attributes["data"].directFrtlMsgIDs
#   @show cliq.attributes["data"].directPriorMsgIDs
#   sp = Gadfly.spy(mat)
#   push!(sp.guides, Gadfly.Guide.title("$(cliq.attributes["label"]) || $(cliq.attributes["data"].frontalIDs) :$(cliq.attributes["data"].separatorIDs)"))
#   push!(sp.guides, Gadfly.Guide.xlabel("fmcmcs $(cliq.attributes["data"].itervarIDs)"))
#   push!(sp.guides, Gadfly.Guide.ylabel("lcl=$(numlcl) || msg=$(size(IIF.getCliqMsgMat(cliq),1))" ))
#   return sp
# end
# function spyCliqMat(bt::BayesTree, lbl::Symbol; showmsg=true)
#   spyCliqMat(IIF.whichCliq(bt,lbl), showmsg=showmsg)
# end
#



tree = wipeBuildNewTree!(fg, drawpdf=true, show=true)


import IncrementalInference: getCliqMat

sym = :x21
whichCliq(tree, sym).attributes["label"]
spyCliqMat(tree, sym)

syms = union(getCliqSymbols(tree, :x21)...)
findfirst(syms, :x21)




getData(whichCliq(tree, :x21))











0













####  DEBUG PPBRDict ====================

LL = [0.6; -16; 0]
function expectedLocalBearing(curPose, LL)
    v = [LL[1] - curPose[1]; LL[2] - curPose[2]]
    world = atan2(v[2], v[1])
    loc = rad2deg(TU.wrapRad(world - curPose[3]))
    while loc < 0
        loc += 360
    end
    return loc
end
# println("$(interp_x[ep]), $(interp_y[ep]), world yaw = $(interp_yaw[ep] * 180 / pi)")
# XX = [interp_x(ep); interp_y[ep]; interp_yaw[ep]]
# expectedLocalBearing(XX, LL)
# println("Expected local yaw = $(expectedLocalBearing(XX, LL)). Sonar local yaw = $(rad2deg(getKDEMax(ppbr.bearing)[1]))")
# xyt = se2vee(SE2(XX[1:3]) \ SE2(LL))
# bear = rad2deg(TU.wrapRad(atan2(xyt[2],xyt[1]) -XX[3]))

bearExp = []
bearAcoustics = []
for ep in epochs
    XX = [interp_x(ep); interp_y[ep]; interp_yaw[ep]]
    ppbr = ppbrDict[ep]
    push!(bearExp, expectedLocalBearing(XX, LL))
    push!(bearAcoustics, rad2deg(getKDEMax(ppbr.bearing)[1]))
    println("local beacon heading: Expected = $(expectedLocalYaw(XX, LL)). Sonar = $(rad2deg(getKDEMax(ppbr.bearing)[1]))")
end

layers = []
push!(layers, Gadfly.layer(x=1:length(bearExp), y=bearExp, Geom.path())[1], Theme(default_color=color("red")))
push!(layers, Gadfly.layer(x=1:length(bearAcoustics), y=bearAcoustics, Geom.path(), Theme(default_color=color("blue"))))
Gadfly.plot(layers...)
# end

####################################



######################################################

# pl = drawPoses(fg, spscale=2.75)
# drawPosesLandms(fg, spscale=0.75) #Means so we don't run into MM == Union() || Dict{} in
# You rolled 20!

# Roll again for debug check
## Debugging strange headings
# posePts = get2DPoseMeans(fg)
# landPts = get2DLandmMeans(fg)
## Debugging landmark bearing range

# ppbrDict[epoch_slice[1]]
# getSample(ppbrDict[epoch_slice[1]],100)
#
#
# addVariable!(fg, :l1, Point2)
# addFactor!(fg, [:x0; :l1], ppbrDict[epoch_slice[1]])
#
# ls(fg, :l1)
# pts = IIF.approxConv(fg, :x0l1f1, :l1)
#
# fct = getData(getVert(fg, :x0l1f1, nt=:fnc))
#
# fct.fnc.zDim

# dev
# ep = epochs[1]
# # azidata[ep][:,1]
# pll = layerBeamPatternRose(ppbrDict[ep].bearing)
# Gadfly.plot(pll...)
#
# pll = layerBeamPatternRose(ppbrDict[ep].bearing, wRr=TU.R(pi/2))
# Gadfly.plot(pll...)

#
# diff(epochs[[end;1]])


## GADFLY EXAMPLE
# plotLocalProduct(fg, :x49, dims=[1;2])
# stuff = IncrementalInference.localProduct(fg, :x49)
# plotKDE(stuff[2], dims=[1;2], levels=10)
#
# # STUFF
# fsym = :x49l1f1
# const TU = TransformUtils
# XX, LL = (KDE.getKDEMax.(IIF.getVertKDE.(fg, IIF.lsf(fg, fsym)))...)
# @show xyt = se2vee(SE2(XX[1:3]) \ SE2([LL[1:2];0.0]))
# bear= rad2deg(TU.wrapRad(atan2(-xyt[2],xyt[1]) -XX[3]))
# b = IncrementalInference.getData(fg, fsym, nt=:fnc).fnc.usrfnc!
# b.bearing
# p = plotKDE(b.range)
# Gadfly.draw(PDF("multimodal_bearing_range.pdf", 12cm, 15cm), p)
#
# # SAVE THIS PLOT
# # epochs = timestamps[11:2:200]
# g = getVertKDE.(fg, [:l1, :x49])
# m1 = KDE.marginal(g[1], [1;2])
# m2 = KDE.marginal(g[2], [1;2])
# norm(diff(KDE.getKDEMax.([m1; m2])))
#
# kde!(rand(Normal(4.0, 0.1),100))
#
# ###############
#
# PL = []
#
# push!(PL, Gadfly.layer(x=collect(1:10), y=randn(10), Geom.path, Theme(default_color=colorant"red"))[1])
# push!(PL, Gadfly.layer(x=collect(1:10), y=randn(10), Geom.path, Theme(default_color=colorant"green"))[1])
# push!(PL, Coord.Cartesian(xmin=-1.0,xmax=11, ymin=-5.0, ymax=5.0))
# push!(PL, Guide.ylabel("test"))
# push!(PL, Guide.ylabel("test"))
# pl = drawPoses(fg)
# push!(pl.layers, )
#
# pl = drawPoses(fg)
# pl = Gadfly.plot(pl.layers...)
#
#
#
# pl = Gadfly.plot(PL...)
#
# @show fieldnames(pl)
# push!(pl.layers, Gadfly.layer(x=collect(1:10), y=randn(10), Geom.path, Theme(default_color=colorant"magenta"))[1] )
#
# pl
# # vstasck, hstack
#
#
# Gadfly.push_theme(:default)
# # SVG, PDF, PNG
# Gadfly.draw(PDF("/tmp/testfig.pdf", 12cm, 8cm), pl)
# run(`evince /tmp/testfig.pdf`)
#
#
# ### PATH + Poses
#
# pl = drawPosesLandms(fg, spscale=2.5)
# # Add odo
# navdf = DataFrame(
#   ts = navkeys,
#   x = X,
#   y = Y
# )
# # pl = Gadfly.layer(navdf, x=:x, y=:y, Geom.path())
# push!(pl.layers, Gadfly.layer(navdf, x=:x, y=:y, Geom.path())[1])
# Gadfly.plot(pl.layers)

fsym = :x49l1f1
const TU = TransformUtils
XX, LL = (KDE.getKDEMax.(IIF.getVertKDE.(fg, IIF.lsf(fg, fsym)))...)
@show xyt = se2vee(SE2(XX[1:3]) \ SE2([LL[1:2];0.0]))
bear= rad2deg(TU.wrapRad(atan2(-xyt[2],xyt[1]) -XX[3]))















#
