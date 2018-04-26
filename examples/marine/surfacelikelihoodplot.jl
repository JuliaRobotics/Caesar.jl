#

using KernelDensityEstimate, KernelDensityEstimatePlotting
using Gadfly
using JLD


toggleYTicks()

inCirc(ptl, cx, cy, r) = norm(ptl - [cx; cy]) < r ? true : false


function inCircPts(ptsl, cx, cy, r)
  len = size(ptsl, 2)
  ret = BitArray(len)
  fill!(ret, true)
  for i in 1:len
    if inCirc(ptsl[:,i], cx, cy, r)
      ret[i] = false
    end
  end
  ret
end

function removeListPoints(ptsl, rmls)
  len = size(ptsl, 2)
  inuse = BitArray(len)
  fill!(inuse, true)
  for i in 1:size(rmls, 1)
    inuse &= inCircPts(ptsl, rmls[i,:]...)
  end
  ptsl[:,inuse]
end


rmlist30 = [
0.8 0.8 0.3;
0 0.7 0.15;
0.5 0 0.2;
0.6 0.5 0.25
]

rmlist10 = [
0.8 0.8 0.5;
0 0.7 0.25;
0.5 0 0.4;
0.6 0.5 0.4
]



N = 300
NN = round(Int, N*0.05)

pts = rand(2, N)
upts30a = removeListPoints(pts, rmlist30)
upts30b = removeListPoints(rand(2,NN), rmlist30)
upts10a = removeListPoints(pts, rmlist10)
upts10b = removeListPoints(rand(2,NN), rmlist10)


pU30 = kde!(upts30a) * kde!(upts30b)
pU10 = kde!(upts10a) #* kde!(upts10b)

pl = plot([pU10; pU30],c=["darkorange"; "darkorchid"], levels=4)

draw(PNG(joinpath(ENV["HOME"], "Pictures", "test.png"),30cm,20cm),pl )


pl10 = plot(pU10, c=["darkorange"], levels=3)
pl30 = plot(pU30, c=["darkorchid"], levels=5)
push!(pl10.layers, pl30.layers[1])
draw(PNG(joinpath(ENV["HOME"], "Pictures", "test2.png"),30cm,20cm),pl10 )

# save, load and redraw the same figure
pts10 = getPoints(pU10)
bw10 = getBW(pU10)[:,1]
pts30 = getPoints(pU30)
bw30 = getBW(pU30)[:,1]
save("/home/dehann/Pictures/testsurface.jld", "pts10", pts10, "bw10", bw10, "pts30", pts30, "bw30", bw30)


d = load("/home/dehann/Pictures/testsurface.jld")

pU10d = kde!(d["pts10"],d["bw10"])
pl10d = plot(pU10d, c=["red"], levels=3)
pU30d = kde!(d["pts30"],d["bw30"])
pl30d = plot(pU30d, c=["seagreen"], levels=5)

push!(pl10d.layers, pl30d.layers[1])


pl10d

draw(PNG(joinpath(ENV["HOME"], "Pictures", "test2d.png"),30cm,20cm),pl10d )
