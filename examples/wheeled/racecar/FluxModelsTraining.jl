# load dfg objects used for training FluxModelsPose2Pose2

# using Revise
using Random
using CuArrays
using Flux
using RoME, IncrementalInference

using Cairo, Fontconfig
using Gadfly, RoMEPlotting
Gadfly.set_default_plot_size(35cm,20cm)


## If distributed plotting is being used

# using Distributed
# addprocs(8)
#
# # using IncrementalInference, Flux, RoME, RoMEPlotting, Gadfly, Cairo, Fontconfig
# # @everywhere using IncrementalInference, Flux, RoME, RoMEPlotting,
#
# using Gadfly, Cairo, Fontconfig
# @everywhere using Gadfly, Cairo, Fontconfig
#
# WP = WorkerPool(setdiff(procs(), [1]))

##

function loadFGsFromList(fgpaths::Vector{<:AbstractString}; trainingNum::Int=0)
  FG = typeof(initfg())[]

  # let FG = FG
  for fgpath in fgpaths
    fg = initfg()
    getSolverParams(fg).logpath = joinpath(splitpath(fgpath)[1:end-1]...,"training_$(trainingNum)")
    mkpath(getLogPath(fg))

    loadDFG(fgpath, Main, fg)

    setShuffleAll!(fg, false)
    setNaiveFracAll!(fg, 0.0)
    enableSolveAllNotDRT!(fg)
    ensureAllInitialized!(fg)

    push!(FG, fg)
  end
  # end

  return FG
end


function plotInterpose(fg::AbstractDFG, prevPs::Symbol, ps::Symbol, fcs::Symbol, lstCount::Int, N::Int)
  nfb = getFactorType(fg, fcs)
  nfb.naiveFrac[] = 0.0
  meas_rot = sampleFluxModelsPose2Pose2(nfb, N, FactorMetadata(), getVariable(fg,prevPs), getVariable(fg,ps))
  # make sure the nfb.joyVelData is set
  pts_dummy = approxConv(fg, fcs, ps, meas_rot)
  pts_pynn = vcat(zeros(2,100), meas_rot[1][3:3,:])
  # @show nfb.joyVelData[1,:]
  for i in 1:N # nfb in NFBs,
    # pts_pynn[1:2,i] = nfb.allPredModels[i](nfb.joyVelData)
    pts_pynn[1:2,i] = nfb.allPredModels[i](nfb.joyVelData)
  end
  # pts = approxConv(fg, fcs, ps, meas)
  nfb.naiveFrac[] = 1.0
  meas = sampleFluxModelsPose2Pose2(nfb, N, FactorMetadata(), getVariable(fg,prevPs), getVariable(fg,ps))
  pts_naive = approxConv(fg, fcs, ps, meas)
  nfb.naiveFrac[] = 0.0
  # get relative value
  pts_slam = zeros(3,100)
  pts_naiv = zeros(3,100)
  x0 = getVal(fg, prevPs)
  x1 = getVal(fg, ps)
  for i in 1:100
    se2vee!((@view pts_slam[:,i]), SE2(x0[:,i])\SE2(x1[:,i]))
    # se2vee!((@view pts_pynn[:,i]), SE2(x0[:,i])\SE2(pts[:,i]))
    se2vee!((@view pts_naiv[:,i]), SE2(x0[:,i])\SE2(pts_naive[:,i]))
  end

  X1slam = manikde!(pts_slam, Pose2)
  X1pynn = manikde!(pts_pynn, Pose2)
  X1naiv = manikde!(pts_naiv, Pose2)
  pl = plotKDE([X1slam;X1pynn;X1naiv], dims=[1;2],legend=["slam";"pred";"naiv"],title="labrun ?, $prevPs ->   $ps",levels=6)
  push!(pl.layers, Gadfly.layer(x=[0],y=[0],Geom.point)[1])
  pl.coord = Coord.Cartesian(xmin=-1,xmax=2,ymin=-1.5,ymax=1.5)
  # pl |> PDF(joinLogPath(fg,"predImgs_$lstCount", "pred_$ps.pdf"),15cm,10cm)
  pl, "predImgs_$lstCount", "pred_$ps.pdf"
end


function drawInterposePredictions(fg::AbstractDFG;
                                  N::Int=100,
                                  varList::Vector{Symbol}=ls(fg, r"x\d") |> sortDFG,
                                  runNumber=0)
  #
  allfiles = readdir(getLogPath(fg))
  lstCount = findall(x->occursin(r"predImgs", x), allfiles) |> length
  mkpath(joinLogPath(fg, "predImgs_$lstCount"))
  prevPs = varList[1]
  taskList = Vector{Task}()
  # let prevPs = prevPs
  for ps in varList[2:end]
    @show fcs = string(prevPs,ps,"f1") |> Symbol
    ts = Threads.@spawn plotInterpose(fg, $prevPs, $ps, $fcs, $lstCount, $N)
    push!(taskList, ts)
    prevPs = ps
    # if length(taskList) == 1
    #   wait(ts)
    # end
  end
  # end

  println("waiting on all tasks")
  for ts in taskList
    pl,fldr,fn = fetch(ts)
    @show fn
    pl |> PDF(joinLogPath(fg,fldr,"$(runNumber)_"*fn),15cm,10cm)
  end
  println("done waiting on tasks")

  unitelist = [["$(runNumber)_pred_$(x).pdf" for x in varList[2:end]]; "$(runNumber)_z_$lstCount.pdf"]
  workingdir = pwd()
  Base.cd(joinLogPath(fg,"predImgs_$lstCount"))
  run(`pdfunite $unitelist`)
  Base.cd(workingdir)
  showzpath = joinLogPath(fg,"predImgs_$lstCount","$(runNumber)_z_$lstCount.pdf")
  # @async run(`evince $showzpath`)

end


function plotInterposeFromData(fg::AbstractDFG, mda::Tuple, models::Vector, pc::Int, lstCount::Int, runNumber::Int)
  jd = mda[1][pc]
  yd = mda[2][pc]
  @assert length(models) == size(yd,2) "currently require num models and yd size to be the same"
  pred_yd = zeros(2,length(models))
  for i in 1:length(models)
    pred_yd[1:2,i] = models[i](jd)
  end
  X1yd = manikde!(Float64.(yd), Pose2)
  X1pr = manikde!(Float64.(pred_yd), Point2)
  pl = plotKDE([X1yd;X1pr], dims=[1;2],legend=["slam";"pred"],title="labrun=$runNumber, pc=$pc",levels=6)
  push!(pl.layers, Gadfly.layer(x=[0],y=[0],Geom.point)[1])
  pl.coord = Coord.Cartesian(xmin=-1,xmax=2,ymin=-1.5,ymax=1.5)
  @show fpath = joinLogPath(fg,"pred_y_$lstCount", "$(runNumber)_pred_$pc.pdf")
  return pl, fpath
end

function drawInterposeFromData(fg::AbstractDFG,
                               mda::Tuple,
                               models::AbstractVector,
                               lstCount;
                               runNumber=0  )
  #
  mkpath(joinLogPath(fg,"pred_y_$lstCount"))
  unitelist = String[]
  taskList = Vector{Task}()
  for pc in 1:length(mda[1])
    ts = Threads.@spawn plotInterposeFromData(fg, mda, models, $pc, $lstCount, $runNumber)
    push!(taskList, ts)
    # pl |> PDF(joinLogPath(fg,"predImgs_$lstCount", "pred_$ps.pdf"),15cm,10cm)
    push!(unitelist, "$(runNumber)_pred_$(pc).pdf")
  end

  println("waiting on all tasks, $(length(taskList))")
  # asyncTasks = []
  for ts in taskList
    pl, fpath = fetch(ts)
    @show fpath
    pl |> PDF(fpath)
    # gg = (p, f) -> (p |> PDF(f))
    # ts = @async Distributed.remotecall(gg, WP, pl, fpath)
    # push!(asyncTasks, ts)
  end
  # println("waiting on async tasks")
  # @show (x->fetch(x)).(asyncTasks)
  # println("done waiting on tasks")

  push!(unitelist, "$(runNumber)_z_$lstCount.pdf")
  workingdir = pwd()
  Base.cd(joinLogPath(fg,"pred_y_$lstCount"))
  @show pwd()
  run(`pdfunite $unitelist`)
  Base.cd(workingdir)
  showzpath = joinLogPath(fg,"pred_y_$lstCount","$(runNumber)_z_$lstCount.pdf")
  # @async run(`evince $showzpath`)

  nothing
end


# x0->x1 for all poses in factor graph (NFBs)
# NFBs = (x->getFactorType(fg, x)).(fcList)
# models = NFBs[1].allPredModels |> deepcopy
# x = nfb.joyVelData
# y = x1.vals
# k is the number of interpose segments in this data
# k+j are intermediate accumulations of longer chords over poses in trajectory
# cho is the interpose accumulation distance
function loss(x,y, i, models, chord=[3;5;10;20;35;50])
  len = length(x)
  res = 0
  for k in 1:len
    res += sum( (y[k][1:2,i] - models[i](x[k])).^2 )
  end
  # first mse component
  res /= len

  # do chord segments
  # can cheat since XY only odo is linear (add and subtract linearly)
  for cho in chord, k in 1:cho:(len-cho)
    separate = +( ([ y[k+j][1:2,i] - models[i](x[k+j]) for j in 0:(cho-1)])... )
    res += sum( separate.^2 ) # / (len/cho)
  end
  res
end

# function lossALMOST(x,y, i, models, chord=[5;])
#   res = 0
#   for k in 1:5:length(x)
#     if k+4 <= length(x)
#       # individuals
#       for kk in k:k+4
#         res += sum( (y[kk][1:2,i] - models[i](x[kk])).^2 )
#       end
#
#       # can cheat since linear
#       res += sum( (y[k+0][1:2,i] - models[i](x[k+0]) +
#                    y[k+1][1:2,i] - models[i](x[k+1]) +
#                    y[k+2][1:2,i] - models[i](x[k+2]) +
#                    y[k+3][1:2,i] - models[i](x[k+3]) +
#                    y[k+4][1:2,i] - models[i](x[k+4])
#                   ).^2
#                 )
#     end
#   end
#   res/length(x)
# end
#
# function lossOLD(x,y, i, models, chord=[5;])
#   res = 0
#   for k in 1:length(x)
#     res += sum( (y[k][1:2,i]-models[i](x[k])).^2 )
#   end
#   res/length(x)
# end


function assembleInterposeData(FG::AbstractVector)

  MDATA = []
  for fg in FG
    varList = ls(fg, r"x\d") |> sortDFG
    fcList = ls(fg, FluxModelsPose2Pose2) |> sortDFG
    prevPs = varList[1]
    gg = (fg, prevPs, ps) -> approxConv(fg, Symbol("$(prevPs)$(ps)f1"), ps)
    taskList = Task[]
    for ps in varList[2:end]
      ts = Threads.@spawn gg(fg, $prevPs, $ps)
      push!(taskList, ts)
      prevPs = ps
    end
    # wait on a thread results
    (x->fetch(x)).(taskList)

    NFBs = (x->getFactorType(fg, x)).(fcList)
    xs = (x->Float32.(x.joyVelData)).(NFBs)
    ys = Vector{Matrix{Float32}}()
    prevPs = varList[1]
    for ps in varList[2:end]
      pts_ip = zeros(Float32, 3,100)
      for i in 1:100
        x0 = getVal(fg, prevPs)
        x1 = getVal(fg, ps)
        pts_ip[:,i] = se2vee(SE2(Float32.(x0[:,i]))\SE2(Float32.(x1[:,i])))
      end
      prevPs = ps
      push!(ys, pts_ip)
    end
    # ys = (x->Float32.(getVal(fg, x))).(varList[2:end]) # NOTE, cannot be in world coordinates (DUH)

    mdata = (xs,ys)
    push!(MDATA, mdata)
  end

  return MDATA
end



# opt = Descent(0.01)
# opt = Descent(0.001)
# opt = Momentum(0.01)
function trainNewModels(FG::Vector{<:AbstractDFG};
                        iter::Int=0,
                        opt = ADAM(0.1),
                        EPOCHS=50,
                        MDATA=assembleInterposeData(FG),
                        loss::Function=loss,
                        models=nothing)
  #
  # get the models from the first FG only (all factors use the same N models)
  fg = FG[1]
  fcList = ls(fg, FluxModelsPose2Pose2) |> sortDFG
  NFBs = (x->getFactorType(fg, x)).(fcList)

  # same models are used everywhere, after training make sure to reset the weights for all NN objects, not just those in models
  lModels = models == nothing ? deepcopy(NFBs[1].allPredModels) : models

  mkpath(joinLogPath(fg,"loss_$iter"))

  ## Do training
  N = 100

  evalcb(n, io=stdout) = println(io, "$n, $(([loss(mdata..., n, lModels) for mdata in MDATA]))")

  function wrapTraining!(n::Int, lModels, MDATA, opt, EPOCHS)
    fid = open(joinLogPath(fg,"loss_$iter","sample_$n.txt"), "w")
    Flux.@epochs EPOCHS Flux.train!((x,y)->loss(x,y,n, lModels), Flux.params(lModels[n]), MDATA, opt, cb = Flux.throttle(()->evalcb(n, fid), 0.1) )
    close(fid)
    nothing
  end

  taskList = Task[]
  for n in 1:N
    ts = Threads.@spawn wrapTraining!($n, lModels, MDATA, opt, EPOCHS)
    push!(taskList, ts)
  end
  println("Waiting on all training tasks.")
  (x->wait(x)).(taskList)
  println("Done waiting on training tasks.")

  return lModels
end


function updateFluxModelsPose2Pose2All!(fg::AbstractDFG,
                                        models::AbstractVector;
                                        makeCopy::Bool=true,
                                        fcList::AbstractVector{Symbol} = ls(fg, FluxModelsPose2Pose2) |> sortDFG  )
  #
  NFBs = (x->getFactorType(fg, x)).(fcList)
  lModels = makeCopy ? deepcopy(models) : models
  for nfb in NFBs, i in 1:length(lModels)
    @assert length(nfb.allPredModels)==length(lModels) "cannot update prediction models of different lengths"
    nfb.allPredModels[i] = lModels[i]
  end
  # (x->(x.allPredModels = lModels)).(NFBs)
  nothing
end



## update the models in a factor graph object

# Distance 0.5
# fgpaths = [
#   "/tmp/caesar/2020-05-01T04:27:36.467/fg_75_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T04:28:55.212/fg_67_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T04:30:08.258/fg_72_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T04:59:39.114/fg_61_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T05:01:03.337/fg_53_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T05:02:15.57/fg_58_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T05:23:54.609/fg_55_resolve.tar.gz";
#   "/tmp/caesar/2020-05-01T05:25:04.904/fg_55_resolve.tar.gz"
# ]

# distance 0.2
fgpaths = [
  "/tmp/caesar/2020-05-08T13:56:26.606/fg_153_resolve.tar.gz";
  "/tmp/caesar/2020-05-08T13:57:30.049/fg_129_resolve.tar.gz";
   "/tmp/caesar/2020-05-08T13:58:35.61/fg_143_resolve.tar.gz";
  "/tmp/caesar/2020-05-08T14:00:00.995/fg_115_resolve.tar.gz";
  "/tmp/caesar/2020-05-08T14:01:27.805/fg_107_resolve.tar.gz";
  "/tmp/caesar/2020-05-08T14:02:54.472/fg_111_resolve.tar.gz";
  "/tmp/caesar/2020-05-08T14:04:03.479/fg_107_resolve.tar.gz";
  "/tmp/caesar/2020-05-08T14:04:49.405/fg_106_resolve.tar.gz"
]




maxTr = Ref{Int}()
maxTr[] = 0
let maxTr = maxTr
for fgp in fgpaths
  fgdir = dirname(fgp)
  numDirsT = findall(x->occursin(r"training",x), readdir(fgdir)) |> length
  maxTr[] = maximum([maxTr[]; numDirsT])
end
end

FG = loadFGsFromList(fgpaths, trainingNum=maxTr[])

@show maxTr


## final prep

# Human evaluation of a few factors
# get a smaller graph

# varList = ls(FG[1], r"x\d") |> sortDFG
# nfg = IIF.buildSubgraphFromLabels!(FG[1], varList[1:50])
# drawGraph(nfg, show=true)

FITFG = FG[1:6]
MDATA=assembleInterposeData(FITFG)


# get any copy of the models
fcList = ls(FITFG[1], FluxModelsPose2Pose2) |> sortDFG
models = getFactorType(FITFG[1], fcList[1]).allPredModels |> deepcopy

let models=models
for i in 1:length(models)
  theta, re  = Flux.destructure(models[i])
  theta .= randn(Float32, length(theta))
  models[i] = re(theta)
end
end

let FITFG=FITFG, MDATA=MDATA, models=models
  for i in 1:length(FITFG)
    drawInterposeFromData(FITFG[i], MDATA[i], models, 0, runNumber=i)
    # drawInterposePredictions(FITFG[i])
  end
end


## loss(MDATA[1][1], MDATA[1][2], 1, models)

PLOTTASKS = []

let FITFG=FITFG, MDATA=MDATA, models=models, PLOTTASKS=PLOTTASKS
for i in 1:10
  LMDATA=[]
  for j in 1:length(MDATA)
    permlist = (1:length(MDATA[j][1]) |> collect)
    # permlist = shuffle!(1:length(MDATA[j][1]) |> collect)
    push!(LMDATA, (MDATA[j][1][permlist], MDATA[j][2][permlist]) )
  end
  newmodels = trainNewModels(FITFG, iter=i, EPOCHS=30, opt=ADAM(0.1/(0.25*i+0.5)), MDATA=LMDATA, loss=loss, models=models)
  runNum = 0
  for lfg in FITFG
    runNum += 1
    updateFluxModelsPose2Pose2All!(lfg, newmodels)
    # drawInterposePredictions(lfg, runNumber=runNum)
    lfg_, mdata_, newmodels_, i_, runNum_ = deepcopy(lfg), deepcopy(MDATA[runNum]), deepcopy(newmodels), deepcopy(i), deepcopy(runNum)
    ts = @async drawInterposeFromData(lfg_, mdata_, newmodels_, i_, runNumber=runNum_)
    # drawInterposeFromData(lfg, MDATA[runNum], newmodels, i, runNumber=runNum)
    push!(PLOTTASKS, ts)
  end
end
end


##
0

# # Final results
# for lfg in FITFG
#   drawInterposePredictions(lfg, runNumber=9999)
# end

##

#
# loss(x,y, i, models)
#
#
# ## Lets train all models
#
#
# trainNewModels(FG)
#
#
#
# ## update and draw before and afters
#
# tasks = for i in 1:1 #length(FG)
#   drawInterposePredictions(FG[i])
# end
#
#
# updateFluxModelsPose2Pose2!(FG[1], newmodels)
#
# tasks = for i in 1:1 # length(FG)
#   drawInterposePredictions(FG[i])
# end
#
#
# ## Random development stuff
#
# #check first approx and direct eval
#
#
# # meas = sampleFluxModelsPose2Pose2(nfb, N, FactorMetadata(), getVariable(fg,:x0), getVariable(fg,:x1))
#
# NFBs = [getFactorType(fg, :x10x11f1);]
# NFBs[1].joyVelData
# NFBs[1].naiveFrac[] = 0.0
# pts = zeros(2,100)
# let NFBs=NFBs, pts=pts
# for nfb in NFBs, i in 1:N
#   pts[1:2,i] = nfb.allPredModels[i](nfb.joyVelData)
# end
# end
#
# pts
#
#
# pts
#
#
#
# pts = approxConv(fg, :x0x1f1, :x1, meas)
#
# drawPosesLandms(fg, spscale=0.2)
#
# meas
#





#
