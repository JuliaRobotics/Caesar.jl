# hermite

using Zygote
using Gadfly, Colors
Gadfly.set_default_plot_size(35cm, 20cm)

f(x) = exp(-(x^2))

f1(x) = f'(x)
f2(x) = f1'(x)
f3(x) = f2'(x)
# f4(x) = f3'(x) # bridge too far

f3(0)



Gadfly.plot(
Gadfly.layer(y=f.(-5:0.1:5), Geom.path, Theme(default_color=colorant"red")),
Gadfly.layer(y=f1.(-5:0.1:5), Geom.path, Theme(default_color=colorant"green")),
Gadfly.layer(y=f2.(-5:0.1:5), Geom.path, Theme(default_color=colorant"blue")),
Gadfly.layer(y=f3.(-5:0.1:5), Geom.path, Theme(default_color=colorant"magenta")),
# Gadfly.layer(y=f4.(-5:0.1:5), Geom.path, Theme(default_color=colorant"cyan")),
)

#





#
