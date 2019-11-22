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

Gadfly.set_default_plot_size(35cm,25cm)

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
ppbrDict = Dict{Int, Pose2Point2Range}()
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
  # azipts = azidata[ep][:,1]
  # aziprob = kde!(azipts)

  # prep the factor functions
  ppbrDict[ep] = Pose2Point2Range(rangeprob)
  lastepoch = ep
end




## Step: Building the factor graph
fg = initfg()
l1Uncertainty = 0.1
nonParamStep = 1 # Number of poses between nonparametric acoustic factors
# Add a central beacon with a prior
addVariable!(fg, :l1, Point2)
addFactor!(fg, [:l1], IIF.Prior( MvNormal([0.6; -16], l1Uncertainty^2*eye(2)) ))
index = 0
for ep in epochs
    curvar = Symbol("x$index")
    addVariable!(fg, curvar, Pose2)

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

# writeGraphPdf(fg, engine="dot")

ensureAllInitialized!(fg)
batchSolve!(fg)

# drawPosesLandms(fg)
drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY, "Multi-modal ISAM (bearing+range measurement every $nonParamStep poses, l1 uncertainty $l1Uncertainty)")
