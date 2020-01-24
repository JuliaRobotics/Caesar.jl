
using ArgParse
using Caesar, RoME, DistributedFactorGraphs
using DelimitedFiles
using DSP
using Dates
using Glob


include(joinpath(@__DIR__, "CommonUtils.jl"))

#DUPLICATE DEFINITION
# @enum SolverStateMachine SSMReady SSMConsumingSolvables SSMSolving
# @enum HandlerStateMachine HSMReady HSMHandling HSMOverlapHandling HSMBlocking

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        # "--kappa_odo"
        #     help = "Scale the odometry covariance"
        #     arg_type = Float64
        #     default = 1.0
        "--plotSeriesBeliefs"
            help = "Glob fg_* archives and draw belief frames"
            action = :store_true
        "reportDir"
            help = "which folder in which to produce results."
            required = false
    end

    return parse_args(s)
end

pargs = if !isdefined(Main, :parsed_args)
  parse_commandline()
else
  parsed_args
end

# pargs["reportDir"] = "/media/dehann/temp2/caesar/2020-01-23T10:57:18.068/fg_after_x64.tar.gz"
# pargs["reportDir"] = "/tmp/caesar/2020-01-23T13:36:12.392/fg_after_x381.tar.gz"


include(joinpath(@__DIR__, "Plotting.jl"))

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

wtdsh = if !isdefined(Main, :wtdsh)
  # load wtdsh and dashboard
  @show wtFile = string(joinLogPath(fg, "dash.json"))
  wtdshA = readdlm(joinLogPath(fg, "dash.csv"), ',')
  # make array of array
  wtdsh = []
  for i in 1:size(wtdshA,1)
    push!(wtdsh, wtdshA[i,:])
  end
  wtdsh
else
  wtdsh
end

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


# Check how long not solvable tail is?
# map(x->(x,isSolvable(getVariable(fg,x))), getLastPoses(fg, filterLabel=r"x\d", number=15))


# after all initialized
plb = plotSandsharkFromDFG(fg, drawTriads=false, lbls=false)


## add reference layer
posesyms = ls(fg, r"x\d") |> sortDFG
filter!(x->isInitialized(fg, x), posesyms)
filter!(x->solverData(getVariable(fg, x), :lbl) != nothing, posesyms)
XX = (x->(solverData(getVariable(fg, x), :lbl).val[1,1])).(posesyms)
YY = (x->(solverData(getVariable(fg, x), :lbl).val[2,1])).(posesyms)
pl = Gadfly.layer(x=XX, y=YY, Geom.path, Theme(default_color=colorant"magenta"))
union!(plb.layers, pl)
plb |> PDF(joinLogPath(fg,"traj_ref.pdf"))







## plot the DEAD RECKON THREAD



drt_data = readdlm(joinLogPath(fg, "DRT.csv"), ',')

XX = Float64.(drt_data[:,4])
YY = Float64.(drt_data[:,5])

# filter the signals for hard jumps between drt transitions
responsetype = Lowpass(1.0; fs=50)
designmethod = FIRWindow(hanning(64))

# remove those near zero
mask = 40.0 .< (XX.^2 + YY.^2)

XXm = XX[mask]
YYm = YY[mask]
XXf = filt(digitalfilter(responsetype, designmethod), XXm)
YYf = filt(digitalfilter(responsetype, designmethod), YYm)

mask = YYf .< -32
XXfm = XXf[mask]
YYfm = YYf[mask]

# plot DRT
pl = Gadfly.plot(x=XXm[1:10:end], y=YYm[1:10:end], Geom.point, Theme(default_color=colorant"khaki"))
pl |> PDF(joinLogPath(fg,"drt.pdf"))
pl = Gadfly.plot(x=XXf[1:10:end], y=YYf[1:10:end], Geom.point, Theme(default_color=colorant"green"))
pl |> PDF(joinLogPath(fg,"drtf.pdf"))

pl = Gadfly.layer(x=XXfm, y=YYfm, Geom.path, Theme(default_color=colorant"green"))
union!(plb.layers, pl)
pl = Gadfly.layer(x=XXm[mask], y=YYm[mask], Geom.path, Theme(default_color=colorant"khaki"))
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



## plot densities


include(joinpath(@__DIR__, "MakiePlotsFG.jl"))


nvsyms = ls(fg, r"x\d") |> length

# how to suppress window and simply export
pl = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=15, fadeFloor=0.2)

Makie.save(joinLogPath(fg,"fgBeliefs.png"), pl)


# plot a series of frames
if pargs["plotSeriesBeliefs"]
  files = glob("fg_x*.tar.gz", getLogPath(fg))
  indiv = splitpath.(files) .|> x->x[end]
  Base.mkpath(joinLogPath(fg, "frames"))
  global frame = 0
  for ind in indiv
    global frame += 1
    println("frame $frame of $(length(indiv))")
    fname = split(ind, '.')[1]
    fgl = LightDFG{SolverParams}(params=SolverParams())
    loadDFG(joinLogPath(fg, ind), Main, fgl)
    nvars = minimum( [15; length(ls(fgl, r"x\d"))] )
    pl = plotVariableBeliefs(fgl, r"x\d", sortVars=true, fade=nvars, fadeFloor=0.2)
    Makie.save(joinLogPath(fg,"frames/$fname.png"), pl)
  end
end





## Look at factor reports separately
reportFactors(fg, Pose2Point2Range, show=false)


if parsed_args["reportPoses"]
  reportFactors(fg, Pose2Pose2, show=false)
end







0




## Plot reference poses



#
# fg, dashboard = main(logSpeed=0.25, lcm=LCMLog(joinpath(ENV["HOME"],"data","sandshark","lcmlog","lcmlog-2019-11-26.01")) ) # bad ranges

# find . -type f -name '*.pdf' -print0 |
#   while IFS= read -r -d '' file
#     do convert -verbose -density 500 -quality 100 "${file}" "png/${file%.*}.png"
#   done

# ls nobytes/ | sed 's/bt_//g' | sed 's/.pdf//g' | xargs -L1
