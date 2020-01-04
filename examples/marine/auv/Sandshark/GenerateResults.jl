

## draw fg
drawGraph(fg, filepath=joinpath(getLogPath(fg),"fg.pdf"), show=false)
# drawGraph(fg)


##  Draw trajectory & Analyze solve run

plb = plotSandsharkFromDFG(fg, drawTriads=false)
plb |> PDF(joinpath(getLogPath(fg),"traj.pdf"))
# pla = drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblkeys, lblX, lblY)


plc = plotFrontendTiming(wtdsh)
plc |> PDF(joinpath(getLogPath(fg),"timing.pdf"),75cm,25cm)
# accumulated real time slack
wtt = (x->x[6]).(wtdsh)
wtt[end]




# set all to solvable=1

map(x->setSolvable!(fg, x, 1), setdiff(ls(fg), ls(fg, r"drt_\d")))
map(x->setSolvable!(fg, x, 1 ),[lsf(fg, Pose2Pose2);
                                lsf(fg, Pose2Point2Range);
                                lsfPriors(fg)] )
#
ensureAllInitialized!(fg)

saveDFG(fg, joinpath(getLogPath(fg),"fg_final") )

# Check how long not solvable tail is?
# map(x->(x,isSolvable(getVariable(fg,x))), getLastPoses(fg, filterLabel=r"x\d", number=15))


# after all initialized
plb = plotSandsharkFromDFG(fg, drawTriads=false, lbls=false)


## add reference layer
posesyms = ls(fg, r"x\d") |> sortDFG
XX = (x->(getVal(solverData(getVariable(fg, x), :lbl))[1])).(posesyms)
YY = (x->(getVal(solverData(getVariable(fg, x), :lbl))[2])).(posesyms)
pl = Gadfly.layer(x=XX, y=YY, Geom.path, Theme(default_color=colorant"magenta"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref.pdf"))







## plot the DEAD RECKON THREAD



drt_data = readdlm(joinLogPath(fg, "DRT.csv"), ',')

XX = Float64.(drt_data[:,4])
YY = Float64.(drt_data[:,5])

# remove those near zero
mask = 40.0 .< (XX.^2 + YY.^2)

# filter the signals for hard jumps between drt transitions
responsetype = Lowpass(1.0; fs=50)
designmethod = FIRWindow(hanning(64))

XXm = XX[mask]
YYm = YY[mask]
XXf = filt(digitalfilter(responsetype, designmethod), XX[mask])
YYf = filt(digitalfilter(responsetype, designmethod), YY[mask])

pl = Gadfly.plot(x=XXm[1:10:end], y=YYm[1:10:end], Geom.point, Theme(default_color=colorant"khaki"))
pl |> PDF(joinLogPath(fg,"drt.pdf"))
pl = Gadfly.plot(x=XXf[1:10:end], y=YYf[1:10:end], Geom.point, Theme(default_color=colorant"green"))
pl |> PDF(joinLogPath(fg,"drtf.pdf"))

pl = Gadfly.layer(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"green"))
union!(plb.layers, pl)
pl = Gadfly.layer(x=XXm, y=YYm, Geom.path, Theme(default_color=colorant"khaki"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref_drt.pdf"))






odo_data = readdlm(joinLogPath(fg, "ODO.csv"), ',')

XX = Float64.(odo_data[:,3])
YY = Float64.(odo_data[:,4])

# remove those near zero
mask = 40.0 .< (XX.^2 + YY.^2)

XXf = XX #[mask]
YYf = YY #[mask]

pl = Gadfly.plot(x=XXf[1:10:end], y=YYf[1:10:end], Geom.point, Theme(default_color=colorant"black"))
pl |> PDF(joinLogPath(fg,"odo.pdf"))

pl = Gadfly.layer(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"black"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref_drt_odo.pdf"))








odo_data = readdlm(joinLogPath(fg, "RAWODO.csv"), ',')

DX = Float64.(odo_data[:,3:5])

nXYT = devAccumulateOdoPose2(DX, getFactorType(getFactor(fg, :x0f1)).Z.μ)

# remove those near zero for better visualization
mask = 40.0 .< (nXYT[:,1].^2 + nXYT[:,2].^2)

XXf = nXYT[:,1][mask]
YYf = nXYT[:,2][mask]

# pl = Gadfly.plot(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"black"))
pl = plotTrajectoryArrayPose2(nXYT)
pl |> PDF(joinLogPath(fg,"dirodo.pdf"))

pl = Gadfly.layer(x=XXf, y=YYf, Geom.path, Theme(default_color=colorant"black"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref_drt_dirodo.pdf"))






## Look at factors separately
reportFactors(fg, Pose2Point2Range, show=false)

# reportFactors(fg, Pose2Pose2, show=false)


## BATCH SOLVE

# dontMarginalizeVariablesAll!(fg)
# tree, smt, hist = solveTree!(fg)




## Plot reference poses



#
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcmlog-2019-11-26.01")) ) # bad ranges

# find . -type f -name '*.pdf' -print0 |
#   while IFS= read -r -d '' file
#     do convert -verbose -density 500 -quality 100 "${file}" "png/${file%.*}.png"
#   done

# ls nobytes/ | sed 's/bt_//g' | sed 's/.pdf//g' | xargs -L1
