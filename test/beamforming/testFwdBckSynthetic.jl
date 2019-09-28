using Caesar, RoME, IncrementalInference
using DelimitedFiles

# Visualization
using Cairo, Fontconfig
using Colors, Gadfly
using RoMEPlotting, KernelDensityEstimatePlotting


#Array for synthetic data
posData = zeros(10,2);
posData[:,1] = collect(-4.5:4.5);

testDir = joinpath(ENV["HOME"],"data","synthetic","10dB");
csvWaveData = zeros(8192,10);

# gps covariance
rtkCov = Matrix(Diagonal([0.1;0.1].^2))

## build the factor graph
N = 100
fg = initfg()
addVariable!(fg, :l1, Point2)

for i in 1:10
    sym = Symbol("x$i")
    addVariable!(fg, sym, Point2)
    setVal!(fg, sym, zeros(2,N))
    pp = PriorPoint2(MvNormal(posData[i,1:2], rtkCov) )
    addFactor!(fg, [sym;], pp)
end

dataFile = joinpath(testDir,"1.txt")
csvWaveData = readdlm(dataFile,',',Float64,'\n')
#
cfgFile = joinpath(ENV["HOME"],"data","sas","SAS2D.yaml");
cfgd=loadConfigFile(cfgFile)
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");
sas2d = prepareSAS2DFactor(10, csvWaveData, rangemodel=:Correlator, cfgd=cfgd, chirpFile=chirpFile)

#
addFactor!(fg, [:l1;:x1;:x2;:x3;:x4;:x5;:x6;:x7;:x8;:x9;:x10], sas2d, autoinit=false)
#, threadmodel=SingleThreaded )

writeGraphPdf(fg, engine="fdp")

getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = false

tree, smt, hist = solveTree!(fg)

# drawTree(tree, show=true)

# forward SAS case
# for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

@time predL1 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :l1, N=N)
# setVal!(fg, :l1, zeros(2,N))


predL1 = getVal(fg,:l1)
pllim = 800
X1 = getVal(fg, :x1)
pl = Gadfly.plot(
  Gadfly.layer(x=predL1[1,:][-pllim .< predL1[1,:] .< pllim],y=predL1[2,:][-pllim .< predL1[2,:] .< pllim],   Gadfly.Geom.hexbin(xbincount=200, ybincount=200)),
  Gadfly.layer(x=posData[1,:],y=posData[2,:], Gadfly.Geom.point)
); # for skipping Juno display


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
