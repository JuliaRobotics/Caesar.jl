using Revise
using Caesar, RoME, IncrementalInference, DistributedFactorGraphs
using DelimitedFiles
using Test

# Visualization
using Colors, Gadfly
using RoMEPlotting, KernelDensityEstimatePlotting

#posFile = joinpath(dirname(pathof(Caesar)),"..","test","testdata","test_array_positions.csv");
posFile = joinpath(ENV["HOME"],"data","sas","test_array_positions.csv");
posData = readdlm(posFile,',',Float64,'\n')


posData[:,1] .+= 13.0
posData[:,2] .-= 40.0

# gps covariance
rtkCov = Matrix(Diagonal([0.1;0.1].^2))


## build the factor graph
N = 100
fg = initfg()

#landmark
addVariable!(fg, :l1, Point2)
# setVal!(fg, :l1, zeros(2,N))

global pp = nothing
for i in 1:5
    sym = Symbol("x$i")
    addVariable!(fg, sym, Point2)
    setVal!(fg, sym, zeros(2,N))
    pp = PriorPoint2(MvNormal(posData[i,1:2], rtkCov) )
    addFactor!(fg, [sym;], pp)
    # manual init for now
    #setValKDE!(getVariable(fg,sym), kde!(getSample(pp, N)[1]))
    #getData(fg,sym).initialized = true
end

#dataFile = joinpath(dirname(pathof(Caesar)),"..","test","testdata","test_array_waveforms.csv");
dataFile = joinpath(ENV["HOME"],"data","sas","test_array_waveforms.csv");

csvWaveData = readdlm(dataFile,',',Float64,'\n')
csvWaveData = csvWaveData[:,1:5]
#
cfgFile = joinpath(ENV["HOME"],"data","sas","SAS2D.yaml");
cfgd=loadConfigFile(cfgFile)
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");

sas2d = prepareSAS2DFactor(5, csvWaveData, rangemodel=:Correlator, cfgd=cfgd, chirpFile=chirpFile)


f = addFactor!(fg, [:l1;:x1;:x2;:x3;:x4;:x5], sas2d, autoinit=false)
#, threadmodel=SingleThreaded )

import Base.convert
using JSON2

################################
#### Saving and Loading
################################

# Save it
saveFolder = "/tmp/sasDFG"
saveDFG(fg, saveFolder)

fg2 = initfg();
loadDFG(saveFolder, IncrementalInference,fg2)
tree, smt, hist = solveTree!(fg)


retDFG.solverParams = SolverParams()
#### Comparing
# Enable debugging everywhere
ENV["JULIA_DEBUG"] = "all"
#https://stackoverflow.com/questions/53548681/how-to-enable-debugging-messages-in-juno-julia-editor
d = getData(f).fnc.usrfnc!
dataRet = getData(getFactor(retDFG, f.label)).fnc.usrfnc!
@test Caesar.compare(d, dataRet)

@time predL1 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :l1, N=N)s
