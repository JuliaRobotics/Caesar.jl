# new Sandshark example
# add more julia processes
# nprocs() < 7 ? addprocs(7-nprocs()) : nothing

using Caesar
using Interpolations

using RoMEPlotting
using Gadfly, DataFrames
using ProgressMeter

const TU = TransformUtils

using Pkg
using DelimitedFiles

include(joinpath(Pkg.dir("Caesar"), "examples", "marine", "auv", "Sandshark","Plotting.jl"))


## Load the data
# Data structures are:
#   - epochs
#   - NAV
#   - odoDict
#   - ppbrDict
include(joinpath(Pkg.dir("Caesar"), "examples", "marine", "auv", "Sandshark","DataUtils.jl"))

# Initializing Graff session
config = loadGraffConfig();
config.userId = "Dehann"
config.sessionId = "Sandshark"*replace(string(uuid4()), "-" => "")[1:6]
if (!isSessionExisting())
    println(" -- Session '$(config.sessionId)' doesn't exist for robot '$(config.robotId)', creating it...")
    newSessionRequest = SessionDetailsRequest(config.sessionId, "Sandshark dataset!", "Pose2", false)
    session = addSession(newSessionRequest)
end

## Step: Building the factor graph
fg = initfg()
l1Uncertainty = 0.1
nonParamStep = 1 # Number of poses between nonparametric acoustic factors
# Add a central beacon with a prior
addNode!(fg, :l1, Point2)
addFactor!(fg, [:l1], IIF.Prior( MvNormal([0.6; -16], l1Uncertainty^2*eye(2)) ))
index = 0
for ep in epochs
    curvar = Symbol("x$index")
    addNode!(fg, curvar, Pose2)

    # xi -> l1 - nonparametric factor
    if (index+1) % nonParamStep == 0
        info(" - Adding $curvar->l1 factor...")
        # addFactor!(fg, [curvar; (index < length(epochs)/2 ? :l1 : :l2)], ppbrDict[ep])
        addFactor!(fg, [curvar; :l1], ppbrDict[ep])
    end

    if ep != epochs[1]
      # Odo factor x(i-1) -> xi
      info(" - Adding x$(index-1)->$curvar odo factor")
      addFactor!(fg, [Symbol("x$(index-1)"); curvar], odoDict[ep])
    else
      # Prior to the first pose location (a "GPS" prior)
      initLoc = [interp_x(ep);interp_y(ep);interp_yaw(ep)]
      info(" - Adding a initial location prior at $curvar, $initLoc")
      addFactor!(fg, [curvar], IIF.Prior( MvNormal(initLoc, diagm([0.1;0.1;0.05].^2)) ))
    end
    # Heading partial prior
    info(" - Adding heading prior on $curvar")
    addFactor!(fg, [curvar], RoME.PartialPriorYawPose2(Normal(interp_yaw(ep), deg2rad(3))))
    index+=1
end

writeGraphPdf(fg, engine="dot")

ensureAllInitialized!(fg)
batchSolve!(fg)

drawPosesLandms(fg)


# IIF.wipeBuildNewTree!(fg, drawpdf=true)
# run(`evince bt.pdf`)
# run(`evince /tmp/bt.pdf`)

# And I'll redefine anywhere
# Anywhere I RoME
# Where I lay my head is home
# Carved upon my stone
# My body lies, but still I RoME :D
endDogLeg = [interp_x[epochs[end]]; interp_y[epochs[end]]]
estDogLeg = [get2DPoseMeans(fg)[1][end]; get2DPoseMeans(fg)[2][end]]
endDogLeg - estDogLeg

drawPosesLandms(fg)



ls(fg, :x25)

#To Boldly Believe... The Good, the Bad, and the Unbeliefable
X25 = getVertKDE(fg, :x25)

# i
pts = predictbelief(fg, :x21, [:x20x21f1; :x21l1f1])
plotKDE([kde!(pts);X25], dims=[1;2], levels=1, c=["red";"green"])


pts = predictbelief(fg, :x25, :)
plotKDE([kde!(pts);X25], dims=[1;2], levels=1, c=["red";"green"])
plotKDE([kde!(pts);X25], dims=[3], levels=1, c=["red";"green"])


# Solvery! Roll dice for solvery check
# writeGraphPdf(fg)
# ensureAllInitialized!(fg)
t = string(now())
savejld(fg, file="presolve_$t.jld")
IIF.batchSolve!(fg) #, N=100
savejld(fg, file="postsolve_$t.jld")

# pl = drawPoses(fg, spscale=2.75) # Just for odo plot
# Roll again for inspiration check
## PLOT BEAM PATTERNS
Gadfly.push_theme(:default)
pla = drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblX, lblY)
Gadfly.draw(PDF("sandshark-beacon_$t.pdf", 12cm, 15cm), pla)
Gadfly.draw(PNG("sandshark-beacon_$t.png", 12cm, 15cm), pla)





















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
# addNode!(fg, :l1, Point2)
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
