# ship image processing

using RoME, Distributions
using Images, ImageView, ImageDraw
using FileIO
using FixedPointNumbers
using Gadfly
using RoMEPlotting, KernelDensityEstimatePlotting
using StatsBase
using Colors
using ColorTypes



datadir = "/home/dehann/software/multibeam/data/dataset-1/"
datafile = datadir*"1468970745714629_raw.png"
datafile = datadir*"1468970485675274_raw.png"
datafile = datadir*"1468969086135548_raw.png"



img = load(datafile)
imshow(img)


rangebins, azibins = size(img)

# calibrate to SI
ranges = 2.25+9.0/512.0*(0:rangebins-1)
azis = (14.4 - 0.3*(0.5:1:azibins-0.5)) * pi/180.0






#   Factor graphing


N = 75
fg = initfg()


addVariable!(fg, :x0, Pose2, N=N)
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.001^2*eye(3))))

doautoinit!(fg, :x0)

prevsym = :null
prevsymm = :null

for idx in 1:96

lsym = Symbol("l$(idx)")
addVariable!(fg, lsym, Point2, N=N)

llsym = Symbol("l$(idx+1000)")
# addVariable!(fg, llsym, Point2, N=N)

bss = IIF.AliasingScalarSampler(collect(ranges),  Float64.(img[:,idx]), SNRfloor=0.70)
tr = Pose2Point2BearingRange(Normal(azis[idx], 0.005), bss)
addFactor!(fg, [:x0;lsym], tr)
# addFactor!(fg, [:x0;lsym;llsym], tr, multihypo=[1.0;0.5;0.5])


if prevsym != :null
  pp2 = Point2Point2(MvNormal(zeros(2), 0.1^2*eye(2)))
  addFactor!(fg, [prevsym,lsym], pp2)

  # pp22 = Point2Point2(MvNormal(zeros(2), 0.5^2*eye(2)))
  # addFactor!(fg, [prevsymm,llsym], pp22)
end
prevsym = lsym
prevsymm = llsym

end

# the measurement point
# addVariable!(fg, :l48, Point2)
# bss = IIF.AliasingScalarSampler(collect(ranges),  Float64.(img[:,48]), SNRfloor=0.7)
# tr = Pose2Point2BearingRange(Normal(0.0, 0.005), bss)
# addFactor!(fg, [:x0;:l48], tr)



# @async writeGraphPdf(fg)
#
# pts = IIF.approxConv(fg, :x0l48f1, :l48)


# p1 = plotKDE(kde!(pts), axis=[-1.0 10.0;
#                               -1.0 1.0], levels=8) # , c=["red"]
#


getSolverParams(fg).N = N
@time solveTree!(fg)



drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:max)


drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:mean, xmin=5.3,xmax=5.7, ymin=-2.0,ymax=2.0)



plotKDE(fg, [Symbol("l$i") for i in 0:2:60], levels=1)



# savejld(fg)
# loadjld(fg)





imgc = convert(Array{RGB{N0f8}},img)
# imshow(imgc)
# imgdd = RGB.(imgd,imgd,imgd)


# xy = KDE.getKDEMax(getBelief(fg, :l1))

# imgc = ImageDraw.draw(imgc, CirclePointRadius(48, 256, 2), RGB{N0f8}(1.0,0.0,0.0))


for ll in ls(fg)[2]
  xy = KDE.getKDEMax(getBelief(fg, ll))
  the = atan2(-xy[2],xy[1]) * 180.0/pi
  ran = norm(xy)

  u = round(Int, (ran - 2.25)/(9.0/512.0))
  v = round(Int, (the + 14.4)/(0.3))
  @show v
  v = v .< 2 ? 2 : v
  v = 95 .< v ? 95 : v
  @show v, u

  imgc = ImageDraw.draw(imgc, CirclePointRadius(v, u, 2), RGB{N0f8}(1.0,0.0,0.0))

end

imgc











## DEV WORK








plot(x=ranges, y=img[:,48], Geom.line)


imgf = Float64.(img[:,48])
imgf .-= quantile(imgf, 0.5)
imgf[imgf .< 0.0] = 0.0
imgf ./= norm(imgf)








l1 = layer(x=ranges, y=100*imgf, Geom.line)
plot(l1)


wim = StatsBase.ProbabilityWeights(imgf)


smpls = zeros(100)
StatsBase.alias_sample!(ranges,wim,smpls)



l2 = layer(x=smpls, Geom.histogram(bincount=70), Theme(default_color=colorant"orange"))

plot(l1,l2)









pts = rand(bss, 100)

l3 = layer(x=pts, Geom.histogram(bincount=70), Theme(default_color=colorant"green"))

plot(l1,l2,l3)





#
