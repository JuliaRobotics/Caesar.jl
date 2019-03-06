


## plot range tracking error

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(dferr,
  x=:x, y=:y, color=:Legend, Geom.line,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("range err [m]"),
)

pl |> SVG(joinpath(dirname(@__FILE__),"exports","rangeTrackingError.svg"),10cm,5cm)




##  Plot Ranges

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(df,
  x=:x, y=:y, color=:Legend, Geom.line,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("range [m]"),
)



pl |> SVG(joinpath(dirname(@__FILE__),"exports","trackingRange.svg"),10cm,5cm)



##  Plot angles

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(dfa,
  x=:x, y=:y, color=:Legend, Geom.line,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("angle [rad]"),
)


pl |> SVG(joinpath(dirname(@__FILE__),"exports","trackingAngle.svg"),10cm,5cm)



## Plot Cartesian figure

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(dfcm,
  x=:x, y=:y, color=:Legend, Geom.path,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("horizontal [m]"),
  Guide.ylabel("vertical [m]"),
)

pl.coord=Gadfly.Coord.Cartesian(xmin=-20,xmax=10, ymin=-60,ymax=20)

# Gadfly.set_default_plot_size(10cm, 8cm)
pl |> SVG(joinpath(dirname(@__FILE__),"exports","cartesian.svg"),10cm,6cm)




##  HEATMAP


n = 100
x = LinRange(-5.0, 60.0, n)
y = LinRange(-pi, pi, n)
z = zeros(n,n)

xy = (x,y) -> sum( [TT[i](collect([x y]'))[1] for i in [1:3;5:10;]] )


for i in 1:n, j in 1:n
  z[i,j] = xy(x[i], y[j])
end

z[1,1] -= 0.15

# scene = AbstractPlotting.surface(x, y, fill(0f0, N, N), color = z, shading = false)
scene = AbstractPlotting.contour(x, y, -z, levels = 0, linewidth = 0, fillrange = true )

swit = 301
lines!(scene, randense[1:(swit-1)], angdense[1:(swit-1)], color = :red, linewidth=2.5, linestyle=:dash)
lines!(scene, randense[(swit+1):end], angdense[(swit+1):end], color = :red, linewidth=2.5, linestyle=:dash)



axis = scene[Axis] # get the axis object from the scene
# axis[:grid][:linecolor] = ((:red, 0.5), (:blue, 0.5))
# axis[:names][:textcolor] = ((:red, 1.0), (:blue, 1.0))
axis[:names][:axisnames] = ("range [m]", "angle [rad]")
scene


##

save(joinpath(dirname(@__FILE__),"exports","plot.png"), scene)




## Look at cylindrical product data


sym = :t0
for sym in [Symbol("t$i") for i in 0:(nposes-1)]

pll, plc = plotLocalProductCylinder(fg, sym, show=false, scale=0.1);

# Gadfly.set_default_plot_size(8cm, 6cm)
pll |> SVG(joinpath(dirname(@__FILE__),"exports","localProductLinear$sym.svg"),8cm, 6cm)
plc |> SVG(joinpath(dirname(@__FILE__),"exports","localProductCirc$sym.svg"),8cm, 6cm)

end
