# utility functions needed for sandshark


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
      datadict[parse(Int,split(file,'.')[1])/1] = data
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
    id = round(Int, 1e9*parse(Float64, s[1]))
    # round(Int, 1000 * parse(s[1])) = 1531153292381
    navdata[id] = parse.(Float64,s)
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
    id = round(Int, 1e9*parse(Float64, s[1]))
    if s[2] != "NaN"
        lbldata[id] = parse.(Float64, s)
    end
end
lblkeys = sort(collect(keys(lbldata)))



# function heading2yaw(heading)
# heading = map( x->deg2rad(navdata[x][4]), navkeys )
# wrapheading = TU.wrapRad.(heading)
# end

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

  push!(X, 0.7*getindex(navdata[id],7) ) # 8
  push!(Y, 0.7*getindex(navdata[id],8) ) # 7
  push!(yaw, TU.wrapRad(pi/2-deg2rad(getindex(navdata[id],4))) )  # rotation about +Z
end

lblX = Float64[]
lblY = Float64[]
for id in lblkeys
    push!(lblX, getindex(lbldata[id],2) )
    push!(lblY, getindex(lbldata[id],3) )
end

# Build interpolators for x, y, yaw
interp_x = LinearInterpolation(navkeys, X)
interp_y = LinearInterpolation(navkeys, Y)
interp_yaw = LinearInterpolation(navkeys, yaw)



function poorMansDeconv_BF(XX = collect(range(-pi,pi,length=1000)),
                           YY = ppbrDict[epochs[1]].bearing(XX)      )
  #
  YY1 = YY.^4
  YY1 ./= maximum(YY1)
  YY2 = YY1.^4
  YY2 ./= maximum(YY2)
  # Gadfly.plot(x=XX, y=YY2, Geom.line)
  bss = AliasingScalarSampler(XX, YY2)
  pts = reshape(rand(bss, 200), 1, :)
  pc = manikde!(pts, Sphere1)
  # plotKDECircular(pc)

  return pc
end




# Step: Selecting a subset for processing and build up a cache of the factors.

function doEpochs(timestamps, rangedata, azidata, interp_x, interp_y, interp_yaw, odonoise; TSTART=356, TEND=1200, SNRfloor::Float64=0.6, STRIDE::Int=4)
  #
  ## Caching factors
  ppbrDict = Dict{Int, Pose2Point2BearingRange}()
  ppbDict = Dict{Int, Pose2Point2Bearing}()
  pprDict = Dict{Int, Pose2Point2Range}()
  odoDict = Dict{Int, Pose2Pose2}()
  NAV = Dict{Int, Vector{Float64}}()

  XX = collect(range(-pi,pi,length=1000))

  epochs = timestamps[TSTART:STRIDE:TEND]
  lastepoch = 0
  @showprogress "preparing data" for ep in epochs
    # @show ep
    if lastepoch != 0
      # @show interp_yaw(ep)
      deltaAng = interp_yaw(ep) - interp_yaw(lastepoch)

      wXi = TU.SE2([interp_x(lastepoch);interp_y(lastepoch);interp_yaw(lastepoch)])
      wXj = TU.SE2([interp_x(ep);interp_y(ep);interp_yaw(ep)])
      iDXj = se2vee(wXi\wXj)
      NAV[ep] = iDXj
      # NAV[ep][1:2] .*= 0.7
      # println("$(iDXj[1]), $(iDXj[2]), $(iDXj[3])")

      odoDict[ep] = Pose2Pose2(MvNormal(NAV[ep], odonoise) )
    end
    rangepts = rangedata[ep][:]
    rangeprob = kde!(rangepts)

    # azipts = azidata[ep][:,1]
    azipts = collect(azidata[ep][:,1:1]')
    aziptsw = TU.wrapRad.(azipts)

    # direct
    aziprobl = kde!(azipts)
    # npts = rand(aziprobl, 200)
    # aziprob = manikde!(npts, Sphere1)

    # with deconv
    aziprob = poorMansDeconv_BF(XX, aziprobl(XX))

    # alternative range probability
    rawmf = readdlm("/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/matchedfilter/raw/$(ep).csv",',')
    dvmf = exp.(rawmf[:,2])
    dvmf .= dvmf.^4
    dvmf ./= cumsum(dvmf)[end]
    dvmf .= dvmf.^2
    dvmf ./= cumsum(dvmf)[end]
    range_bss = AliasingScalarSampler(rawmf[:,1], dvmf, SNRfloor=SNRfloor) # exp.(rawmf[:,2])

    # prep the factor functions
    ppbrDict[ep] = Pose2Point2BearingRange(aziprob, range_bss) # rangeprob
    ppbDict[ep] = Pose2Point2Bearing(aziprob) # rangeprob
    pprDict[ep] = Pose2Point2Range(range_bss) # rangeprob
    lastepoch = ep
  end
  return epochs, odoDict, ppbrDict, ppbDict, pprDict, NAV
end
#
