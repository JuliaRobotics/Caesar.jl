# convert dead reckoning back to odo

using Caesar, RoME
using Interpolations
using Distributions

using RoMEPlotting, ApproxManifoldProducts
using Gadfly, Fontconfig, Cairo
Gadfly.set_default_plot_size(35cm,25cm)

using DataFrames
using ProgressMeter
using DelimitedFiles

## load the original data


include(joinpath(@__DIR__,"Plotting.jl"))
include(joinpath(@__DIR__,"SandsharkUtils.jl"))


odonoise = Matrix(Diagonal((20*[0.1;0.1;0.005]).^2))

epochs, odoDict, ppbrDict, ppbDict, pprDict, NAV = doEpochs(timestamps, rangedata, azidata, interp_x, interp_y, interp_yaw, odonoise, TSTART=50, TEND=1200, SNRfloor=0.001, STRIDE=1)

# Build interpolators for x, y from LBL data
itpl_lblx = LinearInterpolation(lblkeys, lblX)
itpl_lbly = LinearInterpolation(lblkeys, lblY)

# # quick look at initial location
# itpl_lblx[epochs[1]]
# itpl_lbly[epochs[1]]
# interp_yaw[epochs[1]]
#
# DeltaX = itpl_lblx[epochs[1]] - interp_x[epochs[1]]
# DeltaY = itpl_lbly[epochs[1]] - interp_y[epochs[1]]


## differentiate odoDict back to deltas only


newSampleRate = 50
totT = (-(epochs[1]-epochs[end])*1e-9)
odoCount = round(Int,totT*newSampleRate)
odoT = range(epochs[1], epochs[end], length=odoCount) |> collect

rsmpX = map(t->interp_x(t), odoT)
rsmpY = map(t->interp_y(t), odoT)
rsmpT = map(t->interp_yaw(t), odoT)

rsmpLX = map(t->itpl_lblx(t), odoT)
rsmpLY = map(t->itpl_lbly(t), odoT)

# Gadfly.plot(x=rsmpX, y=rsmpY, Geom.path())
# Gadfly.plot(x=odoT, y=rsmpT, Geom.path())

# Gadfly.plot(x=rsmpLX, y=rsmpLY, Geom.path())


DX = extractDeltaOdo(rsmpX, rsmpY, rsmpT)

# filter put spikes
DX[1, 0.6 .< abs.(DX[1,:])] .= 0.0
DX[2, 0.6 .< abs.(DX[1,:])] .= 0.0
DX[3, 0.6 .< abs.(DX[1,:])] .= 0.0
DX[1, 0.6 .< abs.(DX[2,:])] .= 0.0
DX[2, 0.6 .< abs.(DX[2,:])] .= 0.0
DX[3, 0.6 .< abs.(DX[2,:])] .= 0.0


# Gadfly.plot(y=DX[1,:], Geom.path()) # |> PDF("/tmp/caesar/random/test.pdf");
# Gadfly.plot(y=DX[2,:], Geom.path())
# Gadfly.plot(y=DX[3,:], Geom.path())


## Back to jump free dead reckoning

mpp = MutablePose2Pose2Gaussian(MvNormal([rsmpX[1];rsmpY[1];rsmpT[1]], 1e-3*Matrix(LinearAlgebra.I, 3,3)))
dt = 1.0
nXYT = zeros(3,size(DX,2))
Qc = 1e-6*Matrix(LinearAlgebra.I, 3,3)
for i in 1:size(DX,2)
  RoME.accumulateDiscreteLocalFrame!(mpp,DX[:,i],Qc,dt)
  nXYT[:,i] .= mpp.Zij.μ
end


# Gadfly.plot(x=nXYT[1,:], y=nXYT[2,:], Geom.path()) # |> PDF("/tmp/caesar/random/test.pdf");
# Gadfly.plot(x=odoT, y=nXYT[3,:], Geom.path())


# Gadfly.plot(
#   Gadfly.layer(x=nXYT[1,:], y=nXYT[2,:], Geom.path()),
#   Gadfly.layer(x=rsmpLX, y=rsmpLY, Geom.path(), Theme(default_color=colorant"red"))
# ) |> PDF("/tmp/caesar/random/test.pdf");


# |> PDF("/tmp/caesar/random/test.pdf");
# @async run(`evince /tmp/caesar/random/test.pdf`)


## add gyro bias

gyro_noise = 5e-8*randn(size(DX,2))
gyrobias = cumsum(gyro_noise)
# Gadfly.plot(y=gyrobias, Geom.line)

DXgyro = deepcopy(DX)
DXgyro[3,:] += gyrobias

mpp = MutablePose2Pose2Gaussian(MvNormal([rsmpX[1];rsmpY[1];rsmpT[1]], 1e-3*Matrix(LinearAlgebra.I, 3,3)))
dt = 1.0
nXYTgyrobias = zeros(3,size(DX,2))
Qc = 1e-6*Matrix(LinearAlgebra.I, 3,3)
for i in 1:size(DX,2)
  RoME.accumulateDiscreteLocalFrame!(mpp,DXgyro[:,i],Qc,dt)
  nXYTgyrobias[:,i] .= mpp.Zij.μ
end

# Gadfly.plot(
#   Gadfly.layer(x=nXYTgyrobias[1,:], y=nXYTgyrobias[2,:], Geom.path()),
#   Gadfly.layer(x=rsmpLX, y=rsmpLY, Geom.path(), Theme(default_color=colorant"red"))
# ) # |> PDF("/tmp/caesar/random/example.pdf");


# gfile = joinpath(ENV["HOME"], "Downloads", "gyrobias.csv")
# writedlm(gfile, gyrobias, ',')


## look at bearing and range measurents

ppbrKeys = keys(ppbrDict) |> collect |> sort

## write the new odom values to log file for real time processing.

#
using BotCoreLCMTypes, LCMCore
using Dates
#
#
lcm=LCM()
#

msg = pose_t()
rmsg = raw_t()



ppbrIdx = 1
startT = round(odoT[1]*1e-6)*1e-3 |> unix2datetime
offsetT = now() - startT
for idx in 1:length(odoT)
  global ppbrIdx
  # get next timestamp to combine msgs
  msgstamp = round(odoT[idx]*1e-6)*1e-3 |> unix2datetime
  msgstamp += offsetT
  while now() < msgstamp
    sleep(0.001)
  end
  msg.utime = round(Int,odoT[idx]*1e-3)
  msg.pos = DX[1:3,idx]
  msg.orientation = [1,0,0,0.0]
  bytes = encode(msg)
  publish(lcm, "AUV_ODOMETRY",bytes)
  # odo with gyro bias
  msg.pos = DXgyro[1:3,idx]
  bytes = encode(msg)
  publish(lcm, "AUV_ODOMETRY_GYROBIAS",bytes)
  # publish ground truth
  msg.pos = [rsmpLX[idx]; rsmpLY[idx]; 0.0]
  bytes = encode(msg)
  publish(lcm, "AUV_LBL_INTERPOLATED",bytes)
  # publish magnetometer (was previously baked in)
  msg.pos = zeros(3)
  qMag = convert(Quaternion, Euler([0,0,rsmpT[idx]]))
  msg.orientation = [qMag.s; qMag.v]
  bytes = encode(msg)
  publish(lcm, "AUV_MAGNETOMETER",bytes)

  ## send possible ranges and bearings too
  epT = ppbrKeys[ppbrIdx]
  rmsgstamp = round(epT*1e-6)*1e-3 |> unix2datetime
  # @show msgstamp - (rmsgstamp + offsetT)
  if (rmsgstamp + offsetT) <= msgstamp
    rmsg.utime = round(Int64, epT*1e-3)
    @show msg.utime, rmsg.utime
    # range data
    data = zeros(length(ppbrDict[epT].range.domain),2)
    data[:,1] = ppbrDict[epT].range.domain
    data[:,2] = ppbrDict[epT].range.weights.values
    rmsg.data = reinterpret(UInt8, vec(reshape(data,:,1))) |> collect
    rmsg.length = length(rmsg.data)
    bytes = encode(rmsg)
    publish(lcm, "AUV_RANGE_CORRL",bytes)
    # bearing data
    rmsg.data = reinterpret(UInt8, getPoints(ppbrDict[epT].bearing)[:]) |> collect
    rmsg.length = length(rmsg.data)
    bytes = encode(rmsg)
    publish(lcm, "AUV_BEARING_CORRL",bytes)

    # advance the counter
    ppbrIdx += 1
  end
end



close(lcm)


#
