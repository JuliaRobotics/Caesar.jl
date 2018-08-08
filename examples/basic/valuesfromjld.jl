# extract estimates from jld file

using Caesar, Distributions, RoME

const KDE = KernelDensityEstimate

datadir = "/home/dehann/Pictures/racecarimgs/2018-08-08T10:29:13.58/"
filename = datadir*"racecar_fg_2018-08-08T10:29:13.58.jld"

fg, = loadjld(file=filename)


# using RoMEPlotting
# drawPosesLandms(fg, spscale=0.1)

xx,ll = ls(fg)


fid = open(datadir*"results_x39.csv","w")
for x in xx
  # @show x,
  mx = KDE.getKDEMean(getVertKDE(fg, x))
  println(fid, "$x, $(mx[1]), $(mx[2]), $(mx[3]), $(mx[4]), $(mx[5])")
end
for x in ll
  # @show x,
  mx = KDE.getKDEMean(getVertKDE(fg, x))
  println(fid, "$x, $(mx[1]), $(mx[2])")
end
close(fid)



#
