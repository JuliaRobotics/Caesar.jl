
stuff = IIF.localProduct(fg, :x5)
stuff[2]

plotKDE([getVertKDE(fg, :x5);stuff[2]], dims=[1;2], levels=3, c=["red";"green";["blue" for i in 1:10]])
pts = KDE.getPoints(stuff[2][1])
Gadfly.plot(x=pts[1,:],y=pts[2,:], Geom.hexbin) |> SVG("/tmp/test.svg", 35cm, 25cm)







#
# setVal!(fg, :l1, predL1)
#
#
# L1 = getVal(fg, :l1)
# pl = plotKDE(kde!(L1))
# # pl.coord = Gadfly.Coord.Cartesian(xmin=-200, xmax=100, ymin=-100, ymax=200);
# pl





# Look at sas2d.dbg
# @show length(sas2d.dbg)
# plot(x=sas2d.dbg.azi_smpls, Geom.histogram)


#
# n=200; pl = plot(x=linspace(0,2pi,length(sas2d.dbg.beams[n])), y=sas2d.dbg.beams[n], Geom.path())
#
#
# n=200; pl = plot(x=linspace(0,2pi,length(sas2d.dbg.beams_cut[n])), y=sas2d.dbg.beams_cut[n], Geom.path())
#




avg_beam_cut = zeros(length(sas2d.dbg.beams_cut[1]))
for i in 1:N
  avg_beam_cut[:] = avg_beam_cut + sas2d.dbg.beams_cut[i]
end
avg_beam_cut ./= sum(avg_beam_cut)




avg_beam = zeros(length(sas2d.dbg.beams[1]))
for i in 1:N
  avg_beam[:] = avg_beam + sas2d.dbg.beams[i]
end
avg_beam ./= sum(avg_beam)






circ_avg_beam = zeros(2,length(avg_beam))

count = 0
for th in linspace(0,2pi,length(avg_beam))
  count += 1
  circ_avg_beam[:,count] = 7500*R(th)*[avg_beam[count];0.0]
end





pl = plot(x=linspace(0,2pi,length(avg_beam)), y=avg_beam, Geom.path(), Coord.Cartesian(ymin=0))
pl = plot(x=linspace(0,2pi,length(avg_beam_cut)), y=avg_beam_cut, Geom.path(), Coord.Cartesian(ymin=0))


# not the median cut beam
plPica = plot(x=circ_avg_beam[1,:], y=circ_avg_beam[2,:], Geom.path(), Theme(default_color=colorant"magenta", line_width=2pt))


plPica.layers = union(plPica.layers, pl.layers)




plPica

0
