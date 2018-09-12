# new Sandshark example
# add more julia processes
nprocs() < 7 ? addprocs(8-nprocs()) : nothing

using Caesar, RoME, KernelDensityEstimate, IncrementalInference
using Interpolations
using Distributions

using RoMEPlotting
using Gadfly, DataFrames
using ProgressMeter

const TU = TransformUtils

datadir = joinpath(ENV["HOME"],"data","sandshark","sample_wombat_2018_09_07","processed","extracted")
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

# heading_to_pi(val)
#     ret = val;
#     ret = 2*pi - ret;
#     ret = ret - 1.5*pi;
#     if ret < -pi
#         ret = ret + 2*pi;
#     end
# end

# GET Y = north,  X = East,  Heading along +Y clockwise [0,360)]
east = Float64[]
north = Float64[]
heading = Float64[]

# WANT X = North,  Y = West,  Yaw is right and rule from +X (0) towards +Y pi/2, [-pi,pi)
# so the drawPoses picture will look flipped from Nicks picture
# remember theta = atan2(y,x)    # this is right hand rule

X = Float64[]
Y = Float64[]
yaw = Float64[]
for id in navkeys
  push!(east, getindex(navdata[id],7)) # x-column csv
  push!(north, getindex(navdata[id],8)) # y-column csv
  push!(heading, getindex(navdata[id],4))
  # push!(yaw, TU.wrapRad(-deg2rad(getindex(navdata[id],4))))

  push!(X, getindex(navdata[id],8) )
  push!(Y, -getindex(navdata[id],7) )
  push!(yaw, TU.wrapRad(-deg2rad(getindex(navdata[id],4))) )  # rotation about +Z
end


# tssubset = navkeys[51:60]
# navdf = DataFrame(
#   ts = navkeys - navkeys[1],
#   x = X,
#   y = Y
# )
# pl = Gadfly.plot(navdf, x=:x, y=:y, Geom.path())
# Gadfly.draw(Gadfly.PNG("/tmp/navss.png", 30cm, 20cm),pl)
#
interp_x = LinearInterpolation(navkeys, X)
interp_y = LinearInterpolation(navkeys, Y)
interp_yaw = LinearInterpolation(navkeys, yaw)

## SELECT SEGMENT OF DATA TO WORK WITH
ppbrDict = Dict{Int, Pose2Point2BearingRange}()
odoDict = Dict{Int, Pose2Pose2}()

# We have 261 timestamps
epochs = timestamps[50:2:261]
NAV = Dict{Int, Vector{Float64}}()
lastepoch = 0
for ep in epochs
  if lastepoch != 0
    # @show interp_yaw(ep)
    deltaAng = interp_yaw(ep) - interp_yaw(lastepoch)
    # wDXij =
    #     [interp_x(ep) - interp_x(lastepoch);
    #      interp_y(ep) - interp_y(lastepoch)]
    wXi = TU.SE2([interp_x(lastepoch);interp_y(lastepoch);interp_yaw(lastepoch)])
    wXj = TU.SE2([interp_x(ep);interp_y(ep);interp_yaw(ep)])
    iDXj = se2vee(wXi\wXj)
    # odoVec = iDXj
    # odoVec = [
    #     mag;
    #     mag;
    #     TransformUtils.wrapRad(deltaAng)]
    NAV[ep] = iDXj
    println("$(iDXj[1]), $(iDXj[2]), $(iDXj[3])")
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

# Sanity check....
for ep in epochs
    x = interp_x(ep)
    y = interp_y(ep)
    yaw = interp_yaw(ep)
    println("$ep, $x, $y, $yaw")
end

## build the factor graph
function buildGraphOdoOnly(epochs, ppbrDict, odoDict, interp_x, interp_y, interp_yaw)::IncrementalInference.FactorGraph
    fg = initfg()

    # Don't care about beacon right now
    # addNode!(fg, :l1, Point2)
    # Pinger location is x=16, y=0.6
    # addFactor!(fg, [curvar], IIF.Prior( MvNormal([16;0.6], diagm([0.1;0.1].^2)) ))

    index = 0
    for ep in epochs
        curvar = Symbol("x$index")
        addNode!(fg, curvar, Pose2)
        # addFactor!(fg, [curvar; :l1], ppbrDict[ep])
        if ep != epochs[1]
          addFactor!(fg, [Symbol("x$(index-1)"); curvar], odoDict[ep])
        else
          # add a prior to the first pose location (a "GPS" prior)
          println("Adding a prior at $curvar, x=$(interp_x(ep)), y=$(interp_y(ep)), yaw=$(interp_yaw(ep))")
          addFactor!(fg, [curvar], IIF.Prior( MvNormal([interp_x(ep);interp_y(ep);interp_yaw(ep)], diagm([0.1;0.1;0.05].^2)) ))
        end
        index+=1
    end

    return fg
end

## build the factor graph
function buildGraphUsingBeacon(epochs, ppbrDict, odoDict, interp_x, interp_y, interp_yaw)::IncrementalInference.FactorGraph
    fg = initfg()

    # Add a central beacon with a prior
    addNode!(fg, :l1, Point2)
    # Pinger location is x=16, y=0.6
    addFactor!(fg, [:l1], IIF.Prior( MvNormal([0.6; -16], diagm([0.1;0.1].^2)) ))

    index = 0
    for ep in epochs
        curvar = Symbol("x$index")
        addNode!(fg, curvar, Pose2)
        addFactor!(fg, [curvar; :l1], ppbrDict[ep])
        if ep != epochs[1]
          addFactor!(fg, [Symbol("x$(index-1)"); curvar], odoDict[ep])
        else
          # add a prior to the first pose location (a "GPS" prior)
          println("Adding a prior at $curvar, x=$(interp_x(ep)), y=$(interp_y(ep)), yaw=$(interp_yaw(ep))")
          addFactor!(fg, [curvar], IIF.Prior( MvNormal([interp_x(ep);interp_y(ep);interp_yaw(ep)], diagm([0.1;0.1;0.05].^2)) ))
        end
        index+=1
    end

    return fg
end

## build the factor graph
function buildGraphSurveyInBeacon(epochs, ppbrDict, odoDict, interp_x, interp_y, interp_yaw)::IncrementalInference.FactorGraph
    fg = initfg()

    # Add a central beacon, no prior
    addNode!(fg, :l1, Point2)
    # addFactor!(fg, [:l1], IIF.Prior( MvNormal([16;0.6], diagm([0.1;0.1].^2)) ))
    setVal!(fg, :l1, zeros(2,100))

    index = 0
    for ep in epochs
        curvar = Symbol("x$index")
        addNode!(fg, curvar, Pose2)
        addFactor!(fg, [curvar; :l1], ppbrDict[ep])
        if ep != epochs[1]
          addFactor!(fg, [Symbol("x$(index-1)"); curvar], odoDict[ep], autoinit=false)
        end
        index+=1
    end
    # add a prior to the first pose location (a "GPS" prior)
    println("Adding a prior")
    addFactor!(fg, [:x0], IIF.Prior( MvNormal([interp_x(epochs[1]);interp_y(epochs[1]);interp_yaw(epochs[1])], diagm([0.1;0.1;0.05].^2)) ), autoinit=false)

    return fg
end

function layerBeamPatternRose(bear::BallTreeDensity; scale::Float64=1.0, c=colorant"magenta", wRr=TU.R(0.0),wTRr=zeros(2))
  tp = reshape([0:0.01:(2pi);], 1, :)
  belp = scale*bear(tp)
  Gadfly.plot(x=tp, y=belp, Geom.path)
  belRose = zeros(2, length(tp))
  belRose[1,:] = belp

  idx = 0
  for rRc in TU.R.(tp)
    idx += 1
    belRose[:,idx] = wRr*rRc*belRose[:,idx] + wTRr
  end

  Gadfly.layer(x=belRose[1,:], y=belRose[2,:], Geom.path, Theme(default_color=c))
end


# pl = drawPoses(fg, spscale=2.5)
function drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y)
    PLL = []
    xx, = ls(fg)
    idx = 0
    for ep in epochs
      idx += 1
      theta = getKDEMax(getVertKDE(fg, xx[idx]))
      wRr = TU.R(theta[3])
      pll = layerBeamPatternRose(ppbrDict[ep].bearing, wRr=wRr, wTRr=theta[1:2], scale=5.0)
      push!(PLL, pll)
    end

    pllandmarks = drawPosesLandms(fg, spscale=2.5)
    # Add odo
    navdf = DataFrame(
      ts = navkeys,
      x = X,
      y = Y
    )
    # pl = Gadfly.layer(navdf, x=:x, y=:y, Geom.path())
    push!(pllandmarks.layers, Gadfly.layer(navdf, x=:x, y=:y, Geom.path(), Theme(default_color=colorant"red"))[1])
    # Gadfly.plot(pl.layers)

    push!(PLL, Coord.Cartesian(xmin=-150.0,xmax=10.0,ymin=-120.0,ymax=20.0))
    pla = plot([PLL;pllandmarks.layers; ]...)
    return pla
end



# Various graph options - choose one
# fg = buildGraphOdoOnly(epochs, ppbrDict, odoDict, interp_x, interp_y, interp_yaw)
fg = buildGraphUsingBeacon(epochs, ppbrDict, odoDict, interp_x, interp_y, interp_yaw)
# fg = buildGraphSurveyInBeacon(epochs, ppbrDict, odoDict, interp_x, interp_y, interp_yaw)

# Solvery! Roll dice for solvery check
writeGraphPdf(fg)
# ensureAllInitialized!(fg)
IIF.batchSolve!(fg) #, N=100

# Roll again for inspiration check
pl = drawPoses(fg, spscale=2.75)
drawPosesLandms(fg, spscale=0.75) #Means so we don't run into MM == Union() || Dict{} in
# You rolled 20!

# Roll agin for debug check
## Debugging strange headings
posePts = get2DPoseMeans(fg)
landPts = get2DLandmMeans(fg)
## Debugging landmark bearing range

# ppbrDict[epoch_slice[1]]
getSample(ppbrDict[epoch_slice[1]],100)


addNode!(fg, :l1, Point2)
addFactor!(fg, [:x0; :l1], ppbrDict[epoch_slice[1]])

ls(fg, :l1)
pts = IIF.approxConv(fg, :x0l1f1, :l1)

fct = getData(getVert(fg, :x0l1f1, nt=:fnc))

fct.fnc.zDim




## PLOT BEAM PATTERNS

Gadfly.push_theme(:default)
pla = drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y)
Gadfly.draw(PDF("sandshark-beacon.pdf", 12cm, 15cm), pla)
Gadfly.draw(PNG("sandshark-beacon.png", 12cm, 15cm), pla)


# dev
# ep = epochs[1]
# # azidata[ep][:,1]
# pll = layerBeamPatternRose(ppbrDict[ep].bearing)
# Gadfly.plot(pll...)
#
# pll = layerBeamPatternRose(ppbrDict[ep].bearing, wRr=TU.R(pi/2))
# Gadfly.plot(pll...)


diff(epochs[[end;1]])



## GADFLY EXAMPLE


PL = []

push!(PL, Gadfly.layer(x=collect(1:10), y=randn(10), Geom.path, Theme(default_color=colorant"red"))[1])
push!(PL, Gadfly.layer(x=collect(1:10), y=randn(10), Geom.path, Theme(default_color=colorant"green"))[1])
push!(PL, Coord.Cartesian(xmin=-1.0,xmax=11, ymin=-5.0, ymax=5.0))
push!(PL, Guide.ylabel("test"))
push!(PL, Guide.ylabel("test"))
pl = drawPoses(fg)
push!(pl.layers, )



pl = drawPoses(fg)
pl = Gadfly.plot(pl.layers...)



pl = Gadfly.plot(PL...)

@show fieldnames(pl)
push!(pl.layers, Gadfly.layer(x=collect(1:10), y=randn(10), Geom.path, Theme(default_color=colorant"magenta"))[1] )

pl
# vstasck, hstack


Gadfly.push_theme(:default)
# SVG, PDF, PNG
Gadfly.draw(PDF("/tmp/testfig.pdf", 12cm, 8cm), pl)
run(`evince /tmp/testfig.pdf`)


### PATH + Poses

pl = drawPosesLandms(fg, spscale=2.5)
# Add odo
navdf = DataFrame(
  ts = navkeys,
  x = X,
  y = Y
)
# pl = Gadfly.layer(navdf, x=:x, y=:y, Geom.path())
push!(pl.layers, Gadfly.layer(navdf, x=:x, y=:y, Geom.path())[1])
Gadfly.plot(pl.layers)


####  DEBUGGGGG====================

# timestamps[51]
#
# fg_demo = RoME.initfg()
#
# addNode!(fg_demo, :x1, Pose2)
# addFactor!(fg_demo, [:x1], IIF.Prior(MvNormal(zeros(3), 0.01^2*eye(3))))
#
# ensureAllInitialized!(fg_demo)
#
# drawPoses(fg_demo)
#
#
#
#
#
# fg = RoME.initfg()
#
# addNode!(fg, :x51, Pose2)
# addFactor!(fg, [:x51], IIF.Prior(MvNormal([X[1259];Y[1259];yaw[1259]], 0.001^2*eye(3))))
#
# addNode!(fg, :x52, Pose2)
# wXi = TU.SE2([X[1259];Y[1259];yaw[1259]])
# wXj = TU.SE2([X[1269];Y[1269];yaw[1269]])
# iDXj = se2vee(wXi\wXj)
# pp = Pose2Pose2(MvNormal([iDXj...], 0.001^2*eye(3)) )
# addFactor!(fg, [:x51;:x52], pp)
#
# addNode!(fg, :x53, Pose2)
# wXi = TU.SE2([X[1269];Y[1269];yaw[1269]])
# wXj = TU.SE2([X[1279];Y[1279];yaw[1279]])
# iDXj = se2vee(wXi\wXj)
# pp = Pose2Pose2(MvNormal([iDXj...], 0.001^2*eye(3)) )
# addFactor!(fg, [:x52;:x53], pp)
#
# addNode!(fg, :x54, Pose2)
# wXi = TU.SE2([X[1279];Y[1279];yaw[1279]])
# wXj = TU.SE2([X[1289];Y[1289];yaw[1289]])
# iDXj = se2vee(wXi\wXj)
# pp = Pose2Pose2(MvNormal([iDXj...], 0.001^2*eye(3)) )
# addFactor!(fg, [:x53;:x54], pp)
#
# addNode!(fg, :x55, Pose2)
# wXi = TU.SE2([X[1289];Y[1289];yaw[1289]])
# wXj = TU.SE2([X[1299];Y[1299];yaw[1299]])
# iDXj = se2vee(wXi\wXj)
# pp = Pose2Pose2(MvNormal([iDXj...], 0.001^2*eye(3)) )
# addFactor!(fg, [:x54;:x55], pp)
#
# ensureAllInitialized!(fg)
#
# # addNode!(fg, :x56, Pose2)
# # addNode!(fg, :x57, Pose2)
# # addNode!(fg, :x58, Pose2)
# # addNode!(fg, :x59, Pose2)
# # addNode!(fg, :x60, Pose2)
#
# drawPoses(fg, spscale=0.1)
#
# yaw[1259]*180.0/pi
# navdata[navkeys[1259]][4]
# yaw[1259]
#
#
# 0
#
# Gadfly.plot(x=X, x=Y, Geom.path)




#
