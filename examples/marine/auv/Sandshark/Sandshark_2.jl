# new Sandshark example

using Caesar, RoME, KernelDensityEstimate
using Interpolations

using RoMEPlotting
using Gadfly, DataFrames

datadir = joinpath(ENV["HOME"],"data","sandshark","sample_wombat_2018_09_07","processed","extracted")
matcheddir = joinpath(datadir, "matchedfilter", "particles")
beamdir = joinpath(datadir, "beamformer", "particles")
# matchedFilter
# 1531152812000000000.csv
matchedFiles = readdir(matcheddir)
beamFiles = readdir(beamdir)
timestamps = map(a -> parse(a[1:end-4]), matchedFiles)
beamtimestamps = map(a -> parse(a[1:end-4]), beamFiles)
unionTs = intersect(timestamps, beamtimestamps)

# load all data into dictionaries
rangedata = Dict{Int,Array{Float64}}()
azidata = Dict{Int,Array{Float64}}()
# rangeT0 = 1531152812000000000

mints = min(timestamps...)
maxts = max(timestamps...)

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
rangekeys = sort(collect(keys(rangedata)))

azidata = loaddircsvs(beamdir)
azikeys = sort(collect(keys(azidata)))


# NAV data
navdata = Dict{Int, Vector{Float64}}()
navfile = readdlm(joinpath(datadir, "nav_data.csv"))
for row in navfile
    s = split(row, ",")
    id = round(Int, 1000*parse(s[1]))
    # round(Int, 1000 * parse(s[1])) = 1531153292381
    navdata[id] = parse.(s)
end
# navdata[first(keys(navdata))]

navkeys = sort(collect(keys(navdata)))

# latency is crazy - yep keys are timestamps
# NAV colums are X,Y = 7,8
#                lat,long = 9,10
# e.g.
# 10-element Array{Float64,1}:
#    1.53115e9
#   -6.33978
#    0.449772
#  129.62
#    0.650937
#    1.156
#   51.9071
#  -33.1335
#   42.3582
#  -71.087







X = Float64[]
Y = Float64[]
for id in navkeys
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

interp_x(sum(navkeys[1:2])/2)

navkeys[1]
epochs[1]

interp_x(epochs[1])

## SELECT SEGMENT OF DATA TO WORK WITH
ppbrDict = Dict{Int, Pose2Point2BearingRange}()


epochs = rangekeys # [51:60]
GPS = Dict{Int, Vector{Float64}}()
# almost there -- be right back.
for ep in epochs
  x, y = interp_x(ep), interp_y(ep)
  GPS[ep] = [x;y]
  rangepts = rangedata[ep][:]
  rangeprob = kde!(rangepts)
  azipts = azidata[ep][:,1]
  aziprob = kde!(azipts)
  # prep the factor functions
  ppbrDict[ep] = Pose2Point2BearingRange(aziprob, rangeprob)
end


GPS_X[7:8]
GPS_Y[7:8]


## build the factor graph

fg = initfg()

# Add a central beacon
addNode!(fg, :l1, Point2)
# addFactor()
index = 0
for ep in epochs
  # Correct variable type Pose2?
  curvar = Symbol("x$index")
  addNode!(fg, curvar, Pose2)
  addFactor!(fg, [curvar; :l1], ppbrDict[ep]) # that makes sense - gotcha -- I was on odo sorry
  index+=1
end


writeGraphPdf(fg)


batchSolve!(fg)










#
