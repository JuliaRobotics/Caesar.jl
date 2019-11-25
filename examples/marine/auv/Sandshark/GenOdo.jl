# convert dead reckoning back to odo

using Caesar, RoME
using Interpolations
using Distributions

using RoMEPlotting, ApproxManifoldProducts
using Gadfly, Fontconfig, Cairo
using DataFrames
using ProgressMeter
using DelimitedFiles


## load the original data

Gadfly.set_default_plot_size(35cm,25cm)

include(joinpath(@__DIR__,"Plotting.jl"))
include(joinpath(@__DIR__,"SandsharkUtils.jl"))


odonoise = Matrix(Diagonal((20*[0.1;0.1;0.005]).^2))

epochs, odoDict, ppbrDict, ppbDict, pprDict, NAV = doEpochs(timestamps, rangedata, azidata, interp_x, interp_y, interp_yaw, odonoise, TEND=800)


# Build interpolators for x, y from LBL data
itpl_lblx = LinearInterpolation(lblkeys, lblX)
itpl_lbly = LinearInterpolation(lblkeys, lblY)

# quick look at initial location
itpl_lblx[epochs[1]]
itpl_lbly[epochs[1]]
interp_yaw[epochs[1]]

DeltaX = itpl_lblx[epochs[1]] - interp_x[epochs[1]]
DeltaY = itpl_lbly[epochs[1]] - interp_y[epochs[1]]


## differentiate odoDict back to deltas only
