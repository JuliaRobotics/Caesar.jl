# extract estimates from jld file

using Caesar, Distributions, RoME

const KDE = KernelDensityEstimate

datetimestamp = "2018-08-09T00:04:36.373"
datadir = "/home/dehann/Pictures/racecarimgs/$(datetimestamp)/"
filename = datadir*"racecar_fg_x237.jld"

fg, = loadjld(file=filename)


# using RoMEPlotting
# drawPosesLandms(fg, spscale=0.1)

xx,ll = ls(fg)


fid = open(datadir*"results_$(datetimestamp).csv","w")
for x in xx
  # @show x,
  mx = KDE.getKDEMean(getBelief(getVariable(fg, x)))
  println(fid, "$x, $(mx[1]), $(mx[2]), $(mx[3]), $(mx[4]), $(mx[5])")
end
for x in ll
  # @show x,
  mx = KDE.getKDEMean(getBelief(getVariable(fg, x)))
  println(fid, "$x, $(mx[1]), $(mx[2])")
end
close(fid)



#
