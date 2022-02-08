using Caesar, RoME, IncrementalInference
using DelimitedFiles

# Visualization
using Colors, Gadfly
using RoMEPlotting, KernelDensityEstimatePlotting

#posFile = joinpath(pkgdir(Caesar),"test","testdata","test_array_positions.csv");
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

#dataFile = joinpath(pkgdir(Caesar),"test","testdata","test_array_waveforms.csv");
dataFile = joinpath(ENV["HOME"],"data","sas","test_array_waveforms.csv");

csvWaveData = readdlm(dataFile,',',Float64,'\n')
csvWaveData = csvWaveData[:,1:5]
#
cfgFile = joinpath(ENV["HOME"],"data","sas","SAS2D.yaml");
cfgd=loadConfigFile(cfgFile)
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");
sas2d = prepareSAS2DFactor(5, csvWaveData, rangemodel=:Correlator, cfgd=cfgd, chirpFile=chirpFile)



#
addFactor!(fg, [:l1;:x1;:x2;:x3;:x4;:x5], sas2d, autoinit=false)
#, threadmodel=SingleThreaded )


writeGraphPdf(fg, engine="fdp")


# sas2d = getfnctype(getVert(fg, :l1x1x2x3x4x5f1, nt=:fnc)) # deepcopy somewhere in IIF

# writeGraphPdf(fg)
# ls(fg, :l1)
# getVal(fg, :l1)

getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true

tree = solveTree!(fg)

drawTree(tree, show=true)


# forward SAS case
# for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

@time predL1 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :l1, N=N)
# setVal!(fg, :l1, zeros(2,N))



pllim = 800
X1 = getVal(fg, :x1)
pl = Gadfly.plot(
  Gadfly.layer(x=predL1[1,:][-pllim .< predL1[1,:] .< pllim],y=predL1[2,:][-pllim .< predL1[2,:] .< pllim],   Gadfly.Geom.hexbin(xbincount=100, ybincount=100)),
  Gadfly.layer(x=X1[1,:],y=X1[2,:], Gadfly.Geom.hexbin(xbincount=100, ybincount=100))
) #; # for skipping Juno display


pl |> PDF("/tmp/test.pdf"); @async run(`evince /tmp/test.pdf`)
pl |> SVG("/tmp/test.svg", 35cm, 25cm); @async run(`firefox /tmp/test.svg`)



## backwards case

# backwards leave one out case
setVal!(fg, :l1, 1.0*randn(2,N))
setVal!(fg, :x2, rand(MvNormal(posData[2,1:2], 1.0*Matrix{Float64}(Diagonal([1.0;1.0]))),N))
# for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

@time predX2 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :x2, N=N)




pl = Gadfly.plot(
layer(x=predX2[1,:], y=predX2[2,:], Geom.histogram2d(xbincount=500,ybincount=500)),
layer(x=posData[1:5,1],y=posData[1:5,2], Geom.point)
)

pl.coord = Coord.Cartesian(xmin=0,xmax=20,ymin=-50,ymax=-35)
pl |> SVG("/tmp/test.svg", 35cm, 25cm)



pl_wait = plotKDE(kde!(predX2))
pl.layers = [pl.layers; pl_wait.layers]

pl |> SVG("/tmp/test.svg", 35cm, 25cm)

0
