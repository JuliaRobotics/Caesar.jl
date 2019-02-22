

## plots

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(dferr,
  x=:x, y=:y, color=:Legend, Geom.line,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("range err [%]"),
)

pl |> SVG(joinpath(dirname(@__FILE__),"export","trackingError.svg"),10cm,5cm)

pl



##

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(df,
  x=:x, y=:y, color=:Legend, Geom.line,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("range [m]"),
)

Plots.plot(mt, lab=["true";"mean";"max"])






##

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(dfa,
  x=:x, y=:y, color=:Legend, Geom.line,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("angle [rad]"),
)



##

Gadfly.set_default_plot_size(10cm, 5cm)
pl = Gadfly.plot(dfcm,
  x=:x, y=:y, Geom.path,
  style(line_width=0.5mm, point_size=0.2mm),
  Guide.xlabel("time [s]"),
  Guide.ylabel("angle [rad]"),
)






## Plot Cartesian figure



Gadfly.set_default_plot_size(10cm, 8cm)


pl = Gadfly.plot(
layer(x=XYdense[1,:], y=XYdense[2,:], Geom.line),
layer(x=px_mean, y=py_mean, Geom.path, Gadfly.Theme(default_color=colorant"red")),
layer(x=px_max, y=py_max, Geom.path, Gadfly.Theme(default_color=colorant"green")),
)





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


save(joinpath(dirname(@__FILE__),"export","plot.png"), scene)
