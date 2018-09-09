# new Sandshark example

using Caesar, RoME, KernelDensityEstimate
using Interpolations
using Distributions

using RoMEPlotting
using Gadfly, DataFrames
using ProgressMeter

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
# time,pitch,roll,yaw,speed,internal_x,internal_y,internal_lat,internal_long

X = Float64[]
Y = Float64[]
yaw = Float64[]
for id in navkeys
  push!(yaw, getindex(navdata[id],4))
  push!(X, getindex(navdata[id],7))
  push!(Y, getindex(navdata[id],8))
end

# navdf = DataFrame(
#   ts = navkeys - navkeys[1],
#   x = X,
#   y = Y
# )
# pl = Gadfly.plot(navdf, x=:x, y=:y, Geom.path())
# Gadfly.draw(Gadfly.PNG("/tmp/navss.png", 30cm, 20cm),pl)

interp_x = LinearInterpolation(navkeys, X)
interp_y = LinearInterpolation(navkeys, Y)
interp_yaw = LinearInterpolation(navkeys, yaw)

## SELECT SEGMENT OF DATA TO WORK WITH
ppbrDict = Dict{Int, Pose2Point2BearingRange}()
odoDict = Dict{Int, Pose2Pose2}()

epochs = timestamps[51:60]
NAV = Dict{Int, Vector{Float64}}()
lastepoch = 0
@showprogress for ep in epochs
  if lastepoch != 0
    NAV[ep] = [
        interp_x(ep) - interp_x(lastepoch);
        interp_y(ep) - interp_y(lastepoch);
        TransformUtils.wrapRad(deg2rad(interp_yaw(ep) - interp_yaw(lastepoch)))] # Bearing?
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


## build the factor graph
fg = initfg()

# Add a central beacon
addNode!(fg, :l1, Point2)
index = 0
epoch_slice = epochs
for ep in epoch_slice
  # Correct variable type Pose2?
  curvar = Symbol("x$index")
  addNode!(fg, curvar, Pose2)
  # addFactor!(fg, [curvar; :l1], ppbrDict[ep]) # that makes sense - gotcha -- I was on odo sorry
  if ep != epoch_slice[1]
      addFactor!(fg, [Symbol("x$(index-1)") curvar], odoDict[ep])
  else
      # add a prior to the first pose location (a "GPS" prior)
      addFactor!(fg, [curvar], Prior(MvNormal([interp_x(ep);interp_y(ep);interp_yaw(ep)], diagm([0.1;0.1;0.005].^2)))
  end
  index+=1
end


writeGraphPdf(fg)
batchSolve!(fg)










#
