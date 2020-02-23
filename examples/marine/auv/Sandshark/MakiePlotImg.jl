
## Load necessary code

using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
pkg"precompile"


using FileIO


include(joinpath(@__DIR__, "CommonUtils.jl"))
# include(joinpath(@__DIR__, "Plotting.jl"))


## load specific case


pargs = if !isdefined(Main, :parsed_args)
  parse_commandline()
else
  parsed_args
end


pargs["reportDir"] = if !haskey(pargs, "reportDir") && isdefined(Main, :fg)
  getLogPath(fg)
else
  # "/tmp/caesar/2020-02-03T03:07:45.938/fg_after_x1181.tar.gz"
  "/tmp/caesar/2020-02-22T02:21:20.777/fg_after_x781.tar.gz"
end



# @show pargs["reportDir"]
# @show splitpath(pargs["reportDir"])

# load the factor graph needed
fg = if !isdefined(Main, :fg)
  println("going to load fg")
  fg = LightDFG{SolverParams}(params=SolverParams())
  @show pathElem = splitpath(pargs["reportDir"])
  @show getSolverParams(fg).logpath = joinpath(pathElem[1:end-1]...)
  loadDFG(pargs["reportDir"], Main, fg)
  fg
else
  fg
end


# random temp data
drt_data, TTm, XXm, YYm, XXf, YYf = loadResultsDRT(fg)


# load here due to name conflict with Point2
using MakieLayout
using Makie

include(joinpath(@__DIR__, "MakiePlotsFG.jl"))


##

dcx = 1200 # 1920
dcy = 771  # 1080

scene, layout = layoutscene(resolution = (dcx, dcy)) #10,

figaxes = [LAxis(scene) for i in 1:1, j in 1:1]
tightlimits!.(figaxes)
layout[1:1, 1:1] = figaxes


img = rotr90(load(ENV["HOME"]*"/Pictures/CharlesDockScale.png"))
# img = rotr90(load(ENV["HOME"]*"/Pictures/charlesLow.png"))
# img = rotr90(MakieGallery.loadasset("cow.png"))

for ax in figaxes
  image!(ax, img)
end

figaxes[1, 1].title = "Charles River"
# figaxes[1, 1].aspect = DataAspect()

# ??
xlims!(ax2, 0, 260)
ylims!(ax2, 0, 170)
# ax2.aspect = AxisAspect(1180/770)
ax2.aspect = AxisAspect(dcx/dcy)
tightlimits!(ax2)
# ??

ax = figaxes[1,1]

ax.xgridvisible = false
ax.ygridvisible = false
ax.xticklabelsvisible = false
ax.yticklabelsvisible = false
ax.xticksvisible = false
ax.yticksvisible = false
# ax.xspinevisible = false
# ax.yspinevisible = false
# ax.xoppositespinevisible = false
# ax.yoppositespinevisible = false
# ax.xtrimspine = true
# ax.ytrimspine = true


# figaxes[1, 2].title = "AxisAspect(1)"
# figaxes[1, 2].aspect = AxisAspect(1)

scene

## new axis

ax2 = LAxis(scene)

arx = 1282
ary = 775
xlims!(ax2, 0, 260)
ylims!(ax2, 0, 170)
ax2.aspect = AxisAspect(arx/ary)
tightlimits!(ax2)

ax2

# ax2.ytickalign = 100
# ax2.yticklabelalign = :right # doesnt work

layout[1,1] = ax2

##

scl = 5.0
ofsx = 90*scl
ofsy = 145*scl

addLinesBelief!(fg, figaxes[1,1], TTm, scale=scl, origin=(ofsx,ofsy))

scene
##


# resolution=(dcx,dcy)
# pl = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=2, fadeFloor=0.2, scene=figaxes[1,1], scale=scl, origin=(ofsx,750))
pl, Z = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=1, extend=0.0,fadeFloor=0.1, scale=scl, origin=(ofsx,ofsy), N=100,
                            xmin=(0-ofsx)/scl, xmax=(dcx-ofsx)/scl,ymin=(0-ofsy)/scl,ymax=(dcy-ofsy)/scl, scene=figaxes[1,1])
#


scene


##


Makie.save("/tmp/test.png", scene)

# ##
#
#
# Makie.scale!(scene, 0.8, 0.7)

##
# using CairoMakie, AbstractPlotting, FileIO
#
# AbstractPlotting.current_backend[] = CairoMakie.CairoBackend("/tmp/test.svg")
#
# Makie.save("/tmp/test.svg", scene)
#
# open("/tmp/test.svg","w") do io
# show(io, MIME"image/svg+xml"(), scene)
# end


##
