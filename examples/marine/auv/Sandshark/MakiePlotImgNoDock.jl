
## Some defaults if not already defined

DRT = isdefined(Main, :DRT) ? DRT : true
PPE = isdefined(Main, :PPE) ? PPE : true
REF = isdefined(Main, :REF) ? REF : true

##

dcx = 1200 # 1920
dcy = 771  # 1080

global scene
scene, layout = layoutscene(resolution = (dcx, dcy)) #10,

ax2 = LAxis(scene)
ax2.xlabel = "East [meters]"
ax2.ylabel = "North [meters]"

figaxes = [ax2]
tightlimits!.(figaxes)
layout[1:1, 1:1] = figaxes



##

scl =  1.0 #5.0
ofsx = 140.0    # 90*scl
ofsy = 170.0    # 145*scl

addLinesBelief!(fg, figaxes[1,1], TTm, scale=scl, origin=(ofsx,ofsy), drt=DRT, ppe=PPE, ref=REF, maskRef=(Second(420),Second(450)) )

scene
##


# resolution=(dcx,dcy)
# pl = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=2, fadeFloor=0.2, scene=figaxes[1,1], scale=scl, origin=(ofsx,750))
pl, Z = plotVariableBeliefs(fg, r"x\d", colormap=:blues, minColorBase=0.0,
                            sortVars=true, fade=1, extend=0.0,fadeFloor=0.2, scale=scl, origin=(ofsx,ofsy), N=100, scene=figaxes[1,1],
                            xmin=140-ofsx, xmax=240-ofsx,ymin=90-ofsy,ymax=130-ofsy, force = true)
                            # xmin=(0-ofsx)/scl, xmax=(dcx-ofsx)/scl,ymin=(0-ofsy)/scl,ymax=(dcy-ofsy)/scl)
#



scene




##
