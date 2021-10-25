

## Looking at the results
using Plots

ppes = map(v -> getPPE(getVariable(newfg, v)).suggested, ls(newfg))
x = map(ppe -> ppe[1], ppes); y = map(ppe -> ppe[2], ppes); h = map(ppe -> ppe[3], ppes)
Plots.plot(x, y, title="Path Plot", lw=3)


## Stuff
using Optim


cost(tf, im1, im2) = evaluateTransform(im1,im2, tf )


# Plotting
xrange = -100.0:1.0:100.0
hrange = -pi:0.1:pi
val = reshape(
  [sweepx(sweeps[10],sweeps[11],xrange); sweepx(sweep_original[10],sweep_original[11],xrange)],
  length(xrange), 2)
Plots.plot(xrange,val)
# Heading
val = reshape(
  [sweeph(sweeps[10],sweeps[11],hrange); sweeph(sweep_original[10],sweep_original[11],hrange)],
  length(hrange), 2)
Plots.plot(hrange,val)

corr_func = (a,b)->sqrt(sum((a .- 0.5).*(b .- 0.5)))
val = reshape(
  [sweepx(sweeps[10],sweeps[11],xrange,diff_func=corr_func);
  sweepx(sweep_original[10],sweep_original[11],xrange,diff_func=corr_func)],
  length(xrange), 2)
Plots.plot(xrange,val)

## Sweep plotting
# sanity check: identity transform should yield zero cost
# @assert evaluateTransform(sweeps[11],sweeps[11],0.,0.,0.) == 0 "There's error with no transform!"

# let's try small displacements:
# sweepx(im1, im2, xrange) = (x->@show evaluateTransform(im1,im2,x,0.,0.)).(xrange)
# sweepy(im1, im2, yrange) = (y->@show evaluateTransform(im1,im2,0.,y,0.)).(yrange)
# sweeph(im1, im2, hrange) = (h->@show evaluateTransform(im1,im2,0.,0.,h)).(hrange)


# using Plots
# xrange = -10:0.1:10
# hrange = -pi:0.1:pi
# Plots.plot(xrange,sweepx(sweeps[10],sweeps[11],xrange))
# Plots.plot(xrange,sweepy(sweeps[10],sweeps[11],xrange))
# Plots.plot(hrange,sweeph(sweeps[10],sweeps[11],hrange))


# fs10 = imfilter(sweeps[10],Kernel.gaussian(3))
# fs11 = imfilter(sweeps[11],Kernel.gaussian(3))
# ffs10 = imfilter(fs10,Kernel.gaussian(3))
# ffs11 = imfilter(fs11,Kernel.gaussian(3))
#
# Plots.plot(xrange,sweepx(ffs10,ffs11,xrange))
# Plots.plot(xrange,sweepy(fs10,fs11,xrange))
