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

epochs, odoDict, ppbrDict, ppbDict, pprDict, NAV = doEpochs(timestamps, rangedata, azidata, interp_x, interp_y, interp_yaw, odonoise, TSTART=50, TEND=800)


# # Build interpolators for x, y from LBL data
# itpl_lblx = LinearInterpolation(lblkeys, lblX)
# itpl_lbly = LinearInterpolation(lblkeys, lblY)
#
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

rsmpX = map(t->interp_x[t], odoT)
rsmpY = map(t->interp_y[t], odoT)
rsmpT = map(t->interp_yaw[t], odoT)


# Gadfly.plot(x=rsmpX, y=rsmpY, Geom.path())
# Gadfly.plot(x=odoT, y=rsmpT, Geom.path())


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


##


mpp = MutablePose2Pose2Gaussian(MvNormal([rsmpX[1];rsmpY[1];rsmpT[1]], 1e-3*Matrix(LinearAlgebra.I, 3,3)))
dt = 1.0
nXYT = zeros(3,size(DX,2))
Qc = 1e-6*Matrix(LinearAlgebra.I, 3,3)
for i in 1:size(DX,2)
  RoME.accumulateDiscreteLocalFrame!(mpp,DX[:,i],Qc,dt)
  nXYT[:,i] .= mpp.Zij.μ
end


Gadfly.plot(x=nXYT[1,:], y=nXYT[2,:], Geom.path()) # |> PDF("/tmp/caesar/random/test.pdf");
Gadfly.plot(x=odoT, y=nXYT[3,:], Geom.path())




# |> PDF("/tmp/caesar/random/test.pdf");
# @async run(`evince /tmp/caesar/random/test.pdf`)





#
