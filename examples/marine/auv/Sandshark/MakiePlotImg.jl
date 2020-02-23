

##

dcx = 1200 # 1920
dcy = 771  # 1080

global scene
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

ax = figaxes[1,1]
ax.title = "Charles River"
# figaxes[1, 1].aspect = DataAspect()

# ??
xlims!(ax, 0, 260)
ylims!(ax, 0, 170)
# ax2.aspect = AxisAspect(1180/770)
ax.aspect = AxisAspect(dcx/dcy)
tightlimits!(ax)
# ??


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

arx = dcx # 1282
ary = dcy # 775
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
pl, Z = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=12, extend=0.0,fadeFloor=0.1, scale=scl, origin=(ofsx,ofsy), N=100,
                            xmin=(0-ofsx)/scl, xmax=(dcx-ofsx)/scl,ymin=(0-ofsy)/scl,ymax=(dcy-ofsy)/scl, scene=figaxes[1,1])
#


scene



##
