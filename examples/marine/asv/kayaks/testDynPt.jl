using Distributed
addprocs(5)


using Caesar, RoME, Distributions
using RoMEPlotting, Gadfly, KernelDensityEstimatePlotting
using KernelDensityEstimate
using IncrementalInference
using DocStringExtensions
using DelimitedFiles
# using TransformUtils

include(joinpath(@__DIR__,"slamUtils.jl"))

## Default parameters
N = 100
pμ = [0.0,0,1,0]
pσ = Matrix(Diagonal([0.1;0.1;0.1;0.1].^2))

symbolstart = 1;
windowstart = 70;
saswindow = 5;

poses = [Symbol("x$i") for i in symbolstart:(symbolstart+saswindow-1)]
sasframes = collect(windowstart:1:(windowstart+saswindow-1))

dataDir = joinpath(ENV["HOME"],"data","kayaks","20_gps_pos");
posData = importdata_nav(sasframes, datadir=dataDir);
navchecked, errorind = sanitycheck_nav(posData)

cfgFile = joinpath(ENV["HOME"],"data","sas","SAS2D.yaml");
cfgd=loadConfigFile(cfgFile)
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");


## Build the factor graph
fg = initfg();

beacon = :l1
addVariable!(fg, beacon, Point2 )


dposData = deepcopy(posData)
cumulativeDrift!(dposData[4:5,:],[0.0;0],[0.1,0.1])
waveformData = importdata_waveforms(sasframes,2, datadir=dataDir);
tcurrent = windowstart*1_000_000

for sym in poses
  global tcurrent
  addVariable!(fg, sym, DynPoint2(ut=tcurrent))
  tcurrent += 1_000_000
end

priors = [1,4]
#Priors
for i in priors
    xdotp = posData[i+1,1] - posData[i,1];
    ydotp = posData[i+1,2] - posData[i,2];
    dpμ = [posData[i,1];posData[i,2];xdotp;ydotp];
    dpσ = Matrix(Diagonal([0.1;0.1;0.1;0.1].^2))
    pp = DynPoint2VelocityPrior(MvNormal(dpμ,dpσ))
    addFactor!(fg, [poses[i];], pp, autoinit=false)
end

vps = [1,2,3,4]
for i in vps
    xdotp = dposData[i+1,1] - dposData[i,1];
    ydotp = dposData[i+1,2] - dposData[i,2];
    dpμ = [xdotp;ydotp;0;0];
    dpσ = Matrix(Diagonal([0.1;0.1;0.1;0.1].^2))
    vp = VelPoint2VelPoint2(MvNormal(dpμ,dpσ))
    addFactor!(fg, [poses[i];poses[i+1]], vp, autoinit=false)
    # IncrementalInference.doautoinit!(fg,[getVariable(fg,poses[i])])
end

# IncrementalInference.doautoinit!(fg,[getVariable(fg,poses[5])])
sas2d = prepareSAS2DFactor(saswindow, waveformData, rangemodel=:Correlator,
                           cfgd=cfgd, chirpFile=chirpFile)
addFactor!(fg, [beacon;poses], sas2d, autoinit=false)

writeGraphPdf(fg, engine="dot")

getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true

tree, smt, hist = solveTree!(fg)



# Gadfly.push_theme(:default)

PL = [];

for i in 1:5
    L1 = getVal(fg, Symbol("x$(i)"))
    push!(PL, layer(x=L1[1,:],y=L1[2,:], Geom.histogram2d))
end


# push!(PL, approxConvFwdBFlayer(fg, poses, :l1, posData[1,:], 2000 ))

push!(PL, plotKDE(getKDE(getVariable(fg, :l1)), levels=4, layers=true)...)

pl = Gadfly.plot(PL...);
pl.coord = Coord.Cartesian(xmin=10,xmax=30,ymin=-60,ymax=-45)

pl |> PDF("/tmp/test.pdf");  @async run(`evince /tmp/test.pdf`)

plname = "testDP_2pp_3vpvp_init.png"
Gadfly.draw(PNG(plname,1.5*20cm,1.5*20cm),pl)

stuff = IncrementalInference.localProduct(fg, :x1)

pl = plotKDE(stuff[1], dims=[1;2], levels=3, c=["blue"])