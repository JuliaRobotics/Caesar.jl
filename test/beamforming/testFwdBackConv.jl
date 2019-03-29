using Caesar, RoME, IncrementalInference

using DelimitedFiles

# visualization scripts

using Colors
# currently need master branch
using Gadfly
# tags waiting on METADATA
using RoMEPlotting, KernelDensityEstimatePlotting
using Pkg





sasframe = 64
logdir = "/media/data1/data/kayaks/20_parsed/array_positions$(sasframe).csv"  # 85
posData = readdlm(logdir,',',Float64,'\n')

posData[:,1] .+= 13.0
posData[:,2] .-= 40.0

posData

# gps covariance
rtkCov = Matrix(Diagonal([0.1;0.1].^2))




N = 200
fg = initfg()

addVariable!(fg, :x1, Point2)
setVal!(fg, :x1, zeros(2,N))
addVariable!(fg, :x2, Point2)
setVal!(fg, :x2, zeros(2,N))
addVariable!(fg, :x3, Point2)
setVal!(fg, :x3, zeros(2,N))
addVariable!(fg, :x4, Point2)
setVal!(fg, :x4, zeros(2,N))
addVariable!(fg, :x5, Point2)
setVal!(fg, :x5, zeros(2,N))
# addVariable!(fg, :x6, Point2)
# setVal!(fg, :x6, zeros(2,N))
# addVariable!(fg, :x7, Point2)
# setVal!(fg, :x7, zeros(2,N))
# addVariable!(fg, :x8, Point2)
# setVal!(fg, :x8, zeros(2,N))
# addVariable!(fg, :x9, Point2)
# setVal!(fg, :x9, zeros(2,N))


addVariable!(fg, :l1, Point2)
setVal!(fg, :l1, zeros(2,N))
# addFactor!(fg, [:l1], Prior(MvNormal(zeros(2), diagm([100;100]) )) )


global pp = nothing
i = 1
for i in 1:5
    sym = Symbol("x$i")
    pp = PriorPoint2(MvNormal(posData[i,1:2], rtkCov) )
    addFactor!(fg, [sym;], pp)
    # manual init
    setValKDE!(fg, sym, kde!(getSample(pp, N)[1]))
end



wavefile = "/media/data1/data/kayaks/20_parsed/array_waveforms$(sasframe).csv"  # 85
csvWaveData = readdlm(wavefile,',',Float64,'\n')
csvWaveData = csvWaveData[:,1:5]
#
cfgd=loadConfigFile("/home/dehann/.julia/dev/ProprietaryFactors/config/SAS2D.yaml")
chirpFile = "/home/dehann/.julia/dev/ProprietaryFactors/src/beamforming/chirp250.txt"
sas2d = prepareSAS2DFactor(5, csvWaveData, rangemodel=:Correlator, cfgd=cfgd, chirpFile=chirpFile)



#
addFactor!(fg, [:l1;:x1;:x2;:x3;:x4;:x5], sas2d, autoinit=false) #, threadmodel=SingleThreaded ) #;:x6;:x7;:x8;:x9

# sas2d = getfnctype(getVert(fg, :l1x1x2x3x4x5f1, nt=:fnc)) # deepcopy somewhere in IIF

# writeGraphPdf(fg)
# ls(fg, :l1)
# getVal(fg, :l1)



# forward SAS case
for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

@time predL1 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :l1, N=N)
# setVal!(fg, :l1, zeros(2,N))



pllim = 800
X1 = getVal(fg, :x1)
pl = Gadfly.plot(
  Gadfly.layer(x=predL1[1,:][-pllim .< predL1[1,:] .< pllim],y=predL1[2,:][-pllim .< predL1[2,:] .< pllim],   Gadfly.Geom.hexbin(xbincount=100, ybincount=100)),
  Gadfly.layer(x=X1[1,:],y=X1[2,:], Gadfly.Geom.hexbin(xbincount=100, ybincount=100))
)



# backwards case



stuff = IIF.localProduct(fg, :x5)
stuff[2]

plotKDE([getVertKDE(fg, :x5);stuff[2]], dims=[1;2], levels=3, c=["red";"green";["blue" for i in 1:10]])
pts = KDE.getPoints(stuff[2][1])
plot(x=pts[1,:],y=pts[2,:], Geom.hexbin)








setVal!(fg, :l1, predL1)


L1 = getVal(fg, :l1)
pl = plotKDE(kde!(L1))
# pl.coord = Gadfly.Coord.Cartesian(xmin=-200, xmax=100, ymin=-100, ymax=200);
pl





# Look at sas2d.dbg
# @show length(sas2d.dbg)
plot(x=sas2d.dbg.azi_smpls, Geom.histogram)



n=200; pl = plot(x=linspace(0,2pi,length(sas2d.dbg.beams[n])), y=sas2d.dbg.beams[n], Geom.path())


n=200; pl = plot(x=linspace(0,2pi,length(sas2d.dbg.beams_cut[n])), y=sas2d.dbg.beams_cut[n], Geom.path())





avg_beam_cut = zeros(length(sas2d.dbg.beams_cut[1]))
for i in 1:N
  avg_beam_cut[:] = avg_beam_cut + sas2d.dbg.beams_cut[i]
end
avg_beam_cut ./= sum(avg_beam_cut)




avg_beam = zeros(length(sas2d.dbg.beams[1]))
for i in 1:N
  avg_beam[:] = avg_beam + sas2d.dbg.beams[i]
end
avg_beam ./= sum(avg_beam)






circ_avg_beam = zeros(2,length(avg_beam))

count = 0
for th in linspace(0,2pi,length(avg_beam))
  count += 1
  circ_avg_beam[:,count] = 7500*R(th)*[avg_beam[count];0.0]
end





pl = plot(x=linspace(0,2pi,length(avg_beam)), y=avg_beam, Geom.path(), Coord.Cartesian(ymin=0))
pl = plot(x=linspace(0,2pi,length(avg_beam_cut)), y=avg_beam_cut, Geom.path(), Coord.Cartesian(ymin=0))


# not the median cut beam
plPica = plot(x=circ_avg_beam[1,:], y=circ_avg_beam[2,:], Geom.path(), Theme(default_color=colorant"magenta", line_width=2pt))


plPica.layers = union(plPica.layers, pl.layers)




plPica

0







# and now the backwards solve


# backwards leave one out case
setVal!(fg, :l1, 1.0*randn(2,N))
setVal!(fg, :x2, rand(MvNormal(posData[2,1:2], 1.0*eye(2)),N))
for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

@time predX2 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :x2, N=N)




pl = Gadfly.plot(
layer(x=predX2[1,:], y=predX2[2,:], Geom.histogram2d(xbincount=500,ybincount=500)),
layer(x=posData[1:5,1],y=posData[1:5,2], Geom.point)
)

pl.coord = Coord.Cartesian(xmin=0,xmax=20,ymin=-50,ymax=-35)
pl



pl_wait = plotKDE(kde!(predX2))
pl.layers = [pl.layers; pl_wait.layers]

pl

0




# local scope equivalent

function main()




    sasframe = 64
    logdir = "/media/data1/data/kayaks/20_parsed/array_positions$(sasframe).csv"  # 85
    posData = readdlm(logdir,',',Float64,'\n')

    posData[:,1] += 13.0
    posData[:,2] -= 40.0

    posData

    # gps covariance
    rtkCov = diagm([0.1;0.1].^2)




    N = 200
    fg = initfg()

    addVariable!(fg, :x1, Point2)
    setVal!(fg, :x1, zeros(2,N))
    addVariable!(fg, :x2, Point2)
    setVal!(fg, :x2, zeros(2,N))
    addVariable!(fg, :x3, Point2)
    setVal!(fg, :x3, zeros(2,N))
    addVariable!(fg, :x4, Point2)
    setVal!(fg, :x4, zeros(2,N))
    addVariable!(fg, :x5, Point2)
    setVal!(fg, :x5, zeros(2,N))
    # addVariable!(fg, :x6, Point2)
    # setVal!(fg, :x6, zeros(2,N))
    # addVariable!(fg, :x7, Point2)
    # setVal!(fg, :x7, zeros(2,N))
    # addVariable!(fg, :x8, Point2)
    # setVal!(fg, :x8, zeros(2,N))
    # addVariable!(fg, :x9, Point2)
    # setVal!(fg, :x9, zeros(2,N))


    addVariable!(fg, :l1, Point2)
    setVal!(fg, :l1, zeros(2,N))
    # addFactor!(fg, [:l1], Prior(MvNormal(zeros(2), diagm([100;100]) )) )





    pp = nothing
    for i in 1:5
      sym = Symbol("x$i")
      pp = PriorPoint2(MvNormal(posData[i,1:2], rtkCov) )
      addFactor!(fg, [sym;], pp)
      setVal!(fg, sym, getSample(pp, N)[1] )
    end



    wavefile = "/media/data1/data/kayaks/20_parsed/array_waveforms$(sasframe).csv"  # 85
    csvWaveData = readdlm(wavefile,',',Float64,'\n')
    csvWaveData = csvWaveData[:,1:5]
    #
    sas2d = prepareSAS2DFactor(5, csvWaveData, rangemodel=:Correlator)

    #
    addFactor!(fg, [:l1;:x1;:x2;:x3;:x4;:x5], sas2d, autoinit=false) #, threadmodel=SingleThreaded ) #;:x6;:x7;:x8;:x9

    # sas2d = getfnctype(getVert(fg, :l1x1x2x3x4x5f1, nt=:fnc)) # deepcopy somewhere in IIF



    # forward SAS case
    for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

    @time predL1 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :l1, N=N)
    # setVal!(fg, :l1, zeros(2,N))
    @time predL1 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :l1, N=N)



    # backwards leave one out case
    setVal!(fg, :l1, 1.0*randn(2,N))
    setVal!(fg, :x2, rand(MvNormal(posData[2,1:2], 1.0*eye(2)),N))
    # for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end

    @time predX2 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :x2, N=N)
    setVal!(fg, :x2, rand(MvNormal(posData[2,1:2], 1.0*eye(2)),N))
    @time predX2 = IncrementalInference.approxConv(fg, ls(fg, :l1)[1], :x2, N=N)



  nothing
end


main()



#
