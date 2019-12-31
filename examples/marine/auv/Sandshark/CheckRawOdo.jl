# test to see if odometry is accumulated correctly from log

using Caesar, RoME
using RoMEPlotting, ApproxManifoldProducts
using Gadfly, Fontconfig, Cairo
Gadfly.set_default_plot_size(35cm,25cm)
using DelimitedFiles

# additional utils
include(joinpath(@__DIR__,"SandsharkUtils.jl"))



# 2019-12-29T14:25:00.747
logpath = "/tmp/caesar/2019-12-31T02:33:50.784"

rawodolog = readdlm(joinpath(logpath,"RAWODO.csv"),',')

DX = Float64.(rawodolog[:,3:5])


nXYT = devAccumulateOdoPose2(DX)


Gadfly.plot(x=nXYT[:,1], y=nXYT[:,2], Geom.path())
Gadfly.plot(y=nXYT[:,3], Geom.path())



pl = plotTrajectoryArrayPose2(nXYT)





## Check DRT odos



drtlog = readdlm(joinpath(logpath,"DRT.csv"),',')

DX = Float64.(drtlog[:,7:9])

nXYT = devAccumulateOdoPose2(DX)

Gadfly.plot(x=nXYT[:,1], y=nXYT[:,2], Geom.path())
Gadfly.plot(y=nXYT[:,3], Geom.path())


pl = plotTrajectoryArrayPose2(nXYT)



#
