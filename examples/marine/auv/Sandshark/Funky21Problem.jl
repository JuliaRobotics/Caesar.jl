# new Sandshark example
# add more julia processes
# nprocs() < 7 ? addprocs(7-nprocs()) : nothing

using Caesar, RoME, KernelDensityEstimate, IncrementalInference
using Interpolations
using Distributions

using RoMEPlotting
using Gadfly, DataFrames
using ProgressMeter

const TU = TransformUtils

include(joinpath(Pkg.dir("Caesar"), "examples", "marine", "auv", "Sandshark","Plotting.jl"))

# datadir = joinpath(ENV["HOME"],"data","sandshark","sample_wombat_2018_09_07","processed","extracted")
datadir = joinpath(ENV["HOME"],"data","sandshark","full_wombat_2018_07_09","extracted")
matcheddir = joinpath(datadir, "matchedfilter", "particles")
beamdir = joinpath(datadir, "beamformer", "particles")

function loaddircsvs(datadir)
  # https://docs.julialang.org/en/v0.6.1/stdlib/file/#Base.Filesystem.walkdir
  datadict = Dict{Int, Array{Float64}}()
  for (root, dirs, files) in walkdir(datadir)
    # println("Files in $root")
    for file in files
      # println(joinpath(root, file)) # path to files
      data = readdlm(joinpath(root, file),',')
      datadict[parse(Int,split(file,'.')[1])/1000000] = data
    end
  end
  return datadict
end

rangedata = loaddircsvs(matcheddir)
azidata = loaddircsvs(beamdir)
timestamps = intersect(sort(collect(keys(rangedata))), sort(collect(keys(azidata))))


# NAV data
navdata = Dict{Int, Vector{Float64}}()
navfile = readdlm(joinpath(datadir, "nav_data.csv"))
for row in navfile
    s = split(row, ",")
    id = round(Int, 1000*parse(s[1]))
    # round(Int, 1000 * parse(s[1])) = 1531153292381
    navdata[id] = parse.(s)
end
navkeys = sort(collect(keys(navdata)))
# NAV colums are X,Y = 7,8
# lat,long = 9,10
# time,pitch,roll,heading,speed,[Something], internal_x,internal_y,internal_lat,internal_long, yaw_rad

# LBL data - note the timestamps need to be exported as float in future.
lbldata = Dict{Int,  Vector{Float64}}()
lblfile = readdlm(joinpath(datadir, "lbl.csv"))
for row in lblfile
    s = split(row, ",")
    id = round(Int, 1000*parse(s[1]))
    @show s
    if s[2] != "NaN"
        lbldata[id] = parse.(s)
    end
end
lblkeys = sort(collect(keys(lbldata)))


# GET Y = north,  X = East,  Heading along +Y clockwise [0,360)]
# east = Float64[]
# north = Float64[]
# heading = Float64[]

# WANT X = North,  Y = West,  Yaw is right and rule from +X (0) towards +Y pi/2, [-pi,pi)
# so the drawPoses picture will look flipped from Nicks picture
# remember theta = atan2(y,x)    # this is right hand rule
X = Float64[]
Y = Float64[]
yaw = Float64[]
for id in navkeys
  # push!(east, getindex(navdata[id],7)) # x-column csv
  # push!(north, getindex(navdata[id],8)) # y-column csv
  # push!(heading, getindex(navdata[id],4))
  # push!(yaw, TU.wrapRad(-deg2rad(getindex(navdata[id],4))))

  push!(X, getindex(navdata[id],8) )
  push!(Y, -getindex(navdata[id],7) )
  push!(yaw, TU.wrapRad(-deg2rad(getindex(navdata[id],4))) )  # rotation about +Z
end

lblX = Float64[]
lblY = Float64[]
for id in lblkeys
    push!(lblX, getindex(lbldata[id],3) )
    push!(lblY, -getindex(lbldata[id],2) )
end

# Build interpolators for x, y, yaw
interp_x = LinearInterpolation(navkeys, X)
interp_y = LinearInterpolation(navkeys, Y)
interp_yaw = LinearInterpolation(navkeys, yaw)

## Caching factors
ppbrDict = Dict{Int, Pose2Point2BearingRange}()
odoDict = Dict{Int, Pose2Pose2}()
NAV = Dict{Int, Vector{Float64}}()

# Step: Selecting a subset for processing and build up a cache of the factors.
epochs = timestamps[50:2:100]
lastepoch = 0
for ep in epochs
  if lastepoch != 0
    # @show interp_yaw(ep)
    deltaAng = interp_yaw(ep) - interp_yaw(lastepoch)

    wXi = TU.SE2([interp_x(lastepoch);interp_y(lastepoch);interp_yaw(lastepoch)])
    wXj = TU.SE2([interp_x(ep);interp_y(ep);interp_yaw(ep)])
    iDXj = se2vee(wXi\wXj)
    NAV[ep] = iDXj
    # println("$(iDXj[1]), $(iDXj[2]), $(iDXj[3])")

    odoDict[ep] = Pose2Pose2(MvNormal(NAV[ep], diagm([0.1;0.1;0.005].^2)))
  end
  rangepts = rangedata[ep][:]
  rangeprob = kde!(rangepts)
  azipts = azidata[ep][:,1]
  aziprob = kde!(azipts)

  # prep the factor functions
  ppbrDict[ep] = Pose2Point2BearingRange(aziprob, rangeprob)
  lastepoch = ep
end


## Step: Building the factor graph
fg = initfg()
# Add a central beacon with a prior
addVariable!(fg, :l1, Point2)
# Pinger location is (0.6; -16)
addFactor!(fg, [:l1], IIF.Prior( MvNormal([0.6; -16], 0.1^2*eye(2)) ))

index = 0
for ep in epochs
    curvar = Symbol("x$index")
    addVariable!(fg, curvar, Pose2)

    # xi -> l1 - nonparametric factor
    # addFactor!(fg, [curvar; :l1], ppbrDict[ep]) #  #Hierdie lyk soos die nagmerrie

    if ep != epochs[1]
      # Odo factor x(i-1) -> xi
      addFactor!(fg, [Symbol("x$(index-1)"); curvar], odoDict[ep])
    else
      # Prior to the first pose location (a "GPS" prior)
      initLoc = [interp_x(ep);interp_y(ep);interp_yaw(ep)]
      println("Adding a prior at $curvar, $initLoc")
      addFactor!(fg, [curvar], IIF.Prior( MvNormal(initLoc, diagm([0.1;0.1;0.05].^2)) ))
    end
    # Heading partial prior
    # addFactor!(fg, [curvar], RoME.PartialPriorYawPose2(Normal(interp_yaw(ep), deg2rad(3))))
    index+=1
end

# Just adding the first one...
addFactor!(fg, [:x0; :l1], ppbrDict[epochs[1]])

addFactor!(fg, [:x5; :l1], ppbrDict[epochs[6]])

addFactor!(fg, [:x10; :l1], ppbrDict[epochs[11]])

addFactor!(fg, [:x13; :l1], ppbrDict[epochs[14]])

addFactor!(fg, [:x15; :l1], ppbrDict[epochs[16]])

addFactor!(fg, [:x17; :l1], ppbrDict[epochs[18]])
addFactor!(fg, [:x18; :l1], ppbrDict[epochs[19]])
addFactor!(fg, [:x19; :l1], ppbrDict[epochs[20]])
addFactor!(fg, [:x20; :l1], ppbrDict[epochs[21]])
# addFactor!(fg, [:x21; :l1], ppbrDict[epochs[22]]) # breaks it!
addFactor!(fg, [:x22; :l1], ppbrDict[epochs[23]])
addFactor!(fg, [:x23; :l1], ppbrDict[epochs[24]])
addFactor!(fg, [:x24; :l1], ppbrDict[epochs[25]])
addFactor!(fg, [:x25; :l1], ppbrDict[epochs[26]])


plotKDE(ppbrDict[epochs[26]].bearing)
plotKDE(ppbrDict[epochs[26]].range)

plotKDE(ppbrDict[epochs[23]].bearing)
plotKDE(ppbrDict[epochs[23]].range)


plotKDE([ppbrDict[epochs[21]].bearing; ppbrDict[epochs[22]].bearing; ppbrDict[epochs[23]].bearing], c=["red";"black";"green"])
plotKDE([ppbrDict[epochs[21]].range; ppbrDict[epochs[22]].range; ppbrDict[epochs[23]].range], c=["red";"black";"green"])


writeGraphPdf(fg, engine="dot")

ensureAllInitialized!(fg)
batchSolve!(fg)

drawPosesLandms(fg)


# IIF.wipeBuildNewTree!(fg, drawpdf=true)
# run(`evince bt.pdf`)
# run(`evince /tmp/bt.pdf`)


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
#   push!(sp.guides, Gadfly.Guide.title("$(cliq.attributes["label"]) || $(cliq.attributes["data"].frontalIDs) :$(cliq.attributes["data"].conditIDs)"))
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

sym = :x25
# get clique sym is in
whichCliq(tree, sym).attributes["label"]
spyCliqMat(tree, sym)

# get all variables in clique
syms = union(getCliqSymbols(tree, sym)...)
varnum = findfirst(syms, sym)
whichpot = getData(whichCliq(tree, sym)).cliqAssocMat[:,varnum]

# get factor ids
fids = getData(whichCliq(tree, sym)).potentials[whichpot]

# get all factors in clique
fsyms = Symbol[]
for factor in getVert.(fg, fids)
  push!(fsyms, Symbol(factor.label))
end

# get KDEs for the factors
pp = kde!.(approxConv.(fg, fsyms, sym))

# plot the actual KDE
plotKDE(pp, dims=[1;2], levels=2)

plotKDE(pp, dims=[1;2], levels=2, legend=string.(fsyms))


# Colors should not appear more than once
# in  at base/<missing>
# in #plotKDE#13 at KernelDensityEstimatePlotting/src/KernelDensityEstimatePlotting.jl:299
# in Gadfly.Guide.ManualColorKey at Gadfly/src/guide.jl:501

ls(fg, :x25)


getData(fg, :x25f1, nt=:fnc).fnc.usrfnc!



getData(whichCliq(tree, sym))



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
