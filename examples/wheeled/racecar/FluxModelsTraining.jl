# load dfg objects used for training FluxModelsPose2Pose2

# using Revise
using CuArrays
using Flux
using RoME

using Cairo, Fontconfig
using RoMEPlotting
using Gadfly
Gadfly.set_default_plot_size(35cm,20cm)


function loadFGsFromList(fgpaths::Vector{<:AbstractString})
  FG = []

  let FG = FG
  for fgpath in fgpaths
  fg = initfg()
  getSolverParams(fg).logpath = joinpath(splitpath(fgpath)[1:end-1]...,"training")
  mkpath(getLogPath(fg))

  loadDFG(fgpath, Main, fg)

  setShuffleAll!(fg, false)
  setNaiveFracAll!(fg, 0.0)
  enableSolveAllNotDRT!(fg)
  ensureAllInitialized!(fg)

  push!(FG, fg)
  end
  end

  return FG
end

function drawInterposePredictions(fg::AbstractDFG;
                                  N=100)
  #

  mkpath(joinLogPath(fg, "predImgs"))
  varList = (ls(fg, r"x\d") |> sortDFG)
  prevPs = :x0
  let prevPs = prevPs
  for ps in varList[2:end]
    @show fcs = string(prevPs,ps,"f1") |> Symbol
    nfb = getFactorType(fg, fcs)
    nfb.naiveFrac[] = 0.0
    meas_rot = sampleFluxModelsPose2Pose2(nfb, N, FactorMetadata(), getVariable(fg,prevPs), getVariable(fg,ps))
    # make sure the nfb.joyVelData is set
    pts_dummy = approxConv(fg, fcs, ps, meas_rot)
    pts_pynn = vcat(zeros(2,100), meas_rot[1][3:3,:])
    @show nfb.joyVelData[1,:]
    for i in 1:N # nfb in NFBs,
      # pts_pynn[1:2,i] = nfb.allPredModels[i](nfb.joyVelData)
      pts_pynn[1:2,i] = models[i](nfb.joyVelData)
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
    pl |> PDF(joinLogPath(fg,"predImgs", "pred_$ps.pdf"),15cm,10cm)
    prevPs = ps
  end
  end

  unitelist = [["pred_$(x).pdf" for x in varList[2:end]]; "z.pdf"]
  workingdir = pwd()
  Base.cd(joinLogPath(fg,"predImgs"))
  run(`pdfunite $unitelist`)
  Base.cd(workingdir)
  showzpath = joinLogPath(fg,"predImgs","z.pdf")
  @async run(`evince $showzpath`)

end


# x0->x1 for all poses in factor graph (NFBs)
# NFBs = (x->getFactorType(fg, x)).(fcList)
# models = NFBs[1].allPredModels |> deepcopy
# x = nfb.joyVelData
# y = x1.vals
function loss(x,y, i, models)
  res = 0
  for k in 1:length(x)
    res += sum( (y[k][1:2,i]-models[i](x[k])).^2 )
  end
  res/length(x)
end


function assembleInterposeData(FG::Vector{<:AbstractDFG})

  MDATA = []
  for fg in FG
    varList = ls(fg, r"x\d") |> sortDFG
    fcList = ls(fg, FluxModelsPose2Pose2) |> sortDFG
    prevPs = varList[1]
    for ps in varList[2:end]
      pts = approxConv(fg, Symbol("$(prevPs)$(ps)f1"), ps)
      prevPs = ps
    end
    NFBs = (x->getFactorType(fg, x)).(fcList)
    xs = (x->Float32.(x.joyVelData)).(NFBs)
    ys = (x->Float32.(getVal(fg, x))).(varList[2:end])
    mdata = (xs,ys)
    push!(MDATA, mdata)
  end

  return MDATA
end

# opt = Descent(0.01)
# opt = Descent(0.001)
# opt = Momentum(0.01)
function trainNewModels(FG::Vector{<:AbstractDFG};
                        opt = ADAM(0.1),
                        EPOCHS=50,
                        MDATA=assembleInterposeData(FG)  )
  #
  # get the models from the first FG only (all factors use the same N models)
  fg = FG[1]
  fcList = ls(fg, FluxModelsPose2Pose2) |> sortDFG
  NFBs = (x->getFactorType(fg, x)).(fcList)

  # same models are used everywhere, after training make sure to reset the weights for all NN objects, not just those in models
  models = NFBs[1].allPredModels |> deepcopy


  ## Do training
  N = 100

  evalcb(n) = @show n, sum([loss(mdata..., n, models) for mdata in MDATA])
  for n in 1:N
    Flux.@epochs EPOCHS Flux.train!((x,y)->loss(x,y,n, models), Flux.params(models[n]), MDATA, opt, cb = Flux.throttle(()->evalcb(n), 0.1) )
  end

  return models
end


function updateFluxModelsPose2Pose2All!(fg::AbstractDFG,
                                        models::AbstractVector;
                                        makeCopy::Bool=true,
                                        fcList::AbstractVector{Symbol} = ls(fg, FluxModelsPose2Pose2) |> sortDFG  )
  #
  NFBs = (x->getFactorType(fg, x)).(fcList)
  lModels = makeCopy ? deepcopy(models) : models
  (x->(x.allPredModels = lModels)).(NFBs)
  nothing
end



## update the models in a factor graph object

fgpaths = [
  "/tmp/caesar/2020-05-01T04:27:36.467/fg_75_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T04:28:55.212/fg_67_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T04:30:08.258/fg_72_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T04:59:39.114/fg_61_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T05:01:03.337/fg_53_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T05:02:15.57/fg_58_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T05:23:54.609/fg_55_resolve.tar.gz";
  "/tmp/caesar/2020-05-01T05:25:04.904/fg_55_resolve.tar.gz"
]


FG = loadFGsFromList(fgpaths)



## Human evaluation of a few factors





## Lets train all models


trainNewModels(FG)



## update and draw before and afters

tasks = for i in 1:1 #length(FG)
  Threads.@spawn drawInterposePredictions(FG[$i])
end


updateFluxModelsPose2Pose2!(FG[1])

tasks = for i in 1:1 # length(FG)
  Threads.@spawn drawInterposePredictions(FG[$i])
end


## Random development stuff

#check first approx and direct eval


# meas = sampleFluxModelsPose2Pose2(nfb, N, FactorMetadata(), getVariable(fg,:x0), getVariable(fg,:x1))

NFBs = [getFactorType(fg, :x10x11f1);]
NFBs[1].joyVelData
NFBs[1].naiveFrac[] = 0.0
pts = zeros(2,100)
let NFBs=NFBs, pts=pts
for nfb in NFBs, i in 1:N
  pts[1:2,i] = nfb.allPredModels[i](nfb.joyVelData)
end
end

pts


pts



pts = approxConv(fg, :x0x1f1, :x1, meas)

drawPosesLandms(fg, spscale=0.2)

meas






#
