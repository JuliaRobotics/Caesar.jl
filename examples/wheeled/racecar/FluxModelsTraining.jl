# load dfg objects used for training FluxModelsPose2Pose2

using Revise
using CuArrays
using Flux
using RoME


function setShuffleAll!(dfg::AbstractDFG, shuf::Bool)
  fs = lsf(dfg, FluxModelsPose2Pose2)
  (x->getFactorType(dfg, x).shuffle[] = shuf).(fs)
  nothing
end

function setNaiveFracAll!(dfg::AbstractDFG, frac::Real)
  fs = lsf(dfg, FluxModelsPose2Pose2)
  (x->getFactorType(dfg, x).naiveFrac[] = frac).(fs)
  nothing
end

##

fgpath = "/tmp/caesar/2020-04-28T15:07:49.807/fg_75_resolve.tar.gz"

fg = initfg()

loadDFG(fgpath, Main, fg)


##

setShuffleAll!(fg, false)
setNaiveFracAll!(fg, 0.0)


##

fcs = :x0x1f1

# ftt = getFactorType(fg, fcs)
# ftt.naiveFrac
# ftt.shuffle

N = 100

enableSolveAllNotDRT!(fg)

solveFactorMeasurements(fg, fcs, 1)

nfb = getFactorType(fg, fcs)

meas = sampleFluxModelsPose2Pose2(nfb, N, FactorMetadata(), getVariable(fg,:x0), getVariable(fg,:x1))


pts = approxConv(fg, fcs, :x1, meas)





X1 = getKDE(fg, :x1)

plotPose()



#
