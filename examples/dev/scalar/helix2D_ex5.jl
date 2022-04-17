

using RoME

##


cb(fg_, lp) = @show lp, length(ls(fg_))

Qd = diagm( [0.1;0.1;0.05].^2 )
fg = generateGraph_Helix2DSlew!(46, slew_x=2/3, posesperturn=15, radius=10, 
                                      useMsgLikelihoods=false, Qd=Qd, postpose_cb=cb)
#


##

##

using TensorCast
using Gadfly
Gadfly.set_default_plot_size(35cm,25cm)

##

vals = getPPE.(fg, (ls(fg) |> sortDFG), :simulated) .|> x-> x.suggested
@cast XYT[d,p] := vals[p][d]
Gadfly.plot(x=XYT[1,:],y=XYT[2,:], Geom.path)

##