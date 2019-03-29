# Victoria Park

using Caesar, RoME
using RoMEPlotting
using Distributions

# load all the model data
# d = odometry information
# f = laser scanner detections
# MM = multi-modal individual id references
# MMr = reworked to map to only one previous feature
# examplefolder, datafolder
include(joinpath(dirname(@__FILE__),"loadVicPrkData.jl"))

# p = drawFeatTrackers( d[1], f[1] );


T=30 # 1400
fg = RoME.emptyFactorGraph();
idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MMr);


drawPosesLandms(fg)


Graphs.plot(fg.g)


vc = startdefaultvisualization()

visualizeallposes!(vc, fg, drawlandms=false)


# Evaluate the likelihood of a point on the marginal belief of some variable
# note the dimensions must match
function evalLikelihood(fg::FactorGraph, sym::Symbol, point::Vector{Float64})
  p = getVertKDE(fg, sym)
  Ndim(p) == length(point) ? nothing : error("point (dim=$(length(point))) must have same dimension as belief (dim=$(Ndim(p)))")
  evaluateDualTree(p, (point')')[1]
end

# Evaluate the likelihood of an Array{2} of points on the marginal belief of some variable
# note the dimensions must match
function evalLikelihood(fg::FactorGraph, sym::Symbol, points::Array{Float64,2})
  p = getVertKDE(fg, sym)
  Ndim(p) == size(points,1) ? nothing : error("points (dim=$(size(points,1))) must have same dimension as belief (dim=$(Ndim(p)))")
  evaluateDualTree(p, (points))
end


evalLikelihood(fg, :l1, [1.0;0.2])

evalLikelihood(fg, :l1, zeros(2,3))


tree = prepBatchTree!(fg, drawpdf=true);
@time inferOverTree!(fg,tree, N=100);
# @save "fgT1400v09err.jld" fg

# include("examples/dev/ISAMRemoteSolve.jl")
# T=1400
# gtvals = doISAMSolve(d,f,toT=T, savejld=true, MM=MMr)

# @load "test/data/isamdict.jld"
# @load "test/data/fgu.jld"

# include("examples/dev/ISAMRemoteSolve.jl")

T=1400
fg = emptyFactorGraph();
idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MMr);
tree = prepBatchTree!(fg, ordering=:qr,drawpdf=true);
@time inferOverTree!(fg,tree, N=100);
@save "fgT1400v06.jld" fg
FG = FactorGraph[]
TREE= BayesTree[]
push!(FG,deepcopy(fg));
push!(TREE,deepcopy(tree));
for i in 1:2
   @time inferOverTreeR!(fg,tree, N=100);
   push!(FG,deepcopy(fg));
   push!(TREE,deepcopy(tree));
end
for i in 1:2
   @time inferOverTree!(fg,tree, N=100);
   push!(FG,deepcopy(fg));
   push!(TREE,deepcopy(tree));
end

  # drawCompPosesLandm(fg,isamdict, fgu, lbls=false,drawunilm=false)













@load "results/fgT1400_Nightly7_28_perErr0.0.jld"
MMMAP0 = deepcopy(fg2)
UMMLE0d = deepcopy(isamdict)
UMMLE0fg = deepcopy(fgu)
rangesMM0_0 = rangeCompAllPoses(UMMLE0d, UMMLE0fg, MMMAP0)

@load "results/fgT1400_Friday7_29_perErr0.01.jld"
MMMAP1 = deepcopy(fg2)
UMMLE1d = deepcopy(isamdict)
UMMLE1fg = deepcopy(fgu)
rangesMM0_1 = rangeCompAllPoses(UMMLE1d, UMMLE1fg, MMMAP0)
rangesMM0_MM1 = rangeCompAllPoses(MMMAP0, MMMAP1)

@load "results/fgT1400_Friday7_29_perErr0.05.jld"
MMMAP5 = deepcopy(fg2)
UMMLE5d = deepcopy(isamdict)
UMMLE5fg = deepcopy(fgu)
rangesMM0_5 = rangeCompAllPoses(UMMLE5d, UMMLE5fg, MMMAP0)
rangesMM0_MM5 = rangeCompAllPoses(MMMAP0, MMMAP5)

@load "results/fgT1400_Friday7_29_perErr0.1.jld"
MMMAP10 = deepcopy(fg2)
UMMLE10d = deepcopy(isamdict)
UMMLE10fg = deepcopy(fgu)
rangesMM0_10 = rangeCompAllPoses(UMMLE10d, UMMLE10fg, MMMAP0)
rangesMM0_MM10 = rangeCompAllPoses(MMMAP0, MMMAP10)

@load "results/fgT1400_Friday7_29_perErr0.15.jld"
MMMAP15 = deepcopy(fg2)
UMMLE15d = deepcopy(isamdict)
UMMLE15fg = deepcopy(fgu)
rangesMM0_15 = rangeCompAllPoses(UMMLE15d, UMMLE15fg, MMMAP0)
rangesMM0_MM15 = rangeCompAllPoses(MMMAP0, MMMAP15)


@load "results/fgT1400_Friday7_29_perErr0.2.jld"
MMMAP20 = deepcopy(fg2)
UMMLE20d = deepcopy(isamdict)
UMMLE20fg = deepcopy(fgu)
rangesMM0_20 = rangeCompAllPoses(UMMLE20d, UMMLE20fg, MMMAP0)
rangesMM0_MM20 = rangeCompAllPoses(MMMAP0, MMMAP20)




ptsize = 1.8pt
# draw all
lyrs = []
push!(lyrs, Gadfly.layer(y=rangesMM0_0, Geom.line))

push!(lyrs, Gadfly.layer(y=rangesMM0_MM1, Geom.line, Theme(default_color=colorant"black")))
push!(lyrs, Gadfly.layer(y=rangesMM0_1, Geom.line, Theme(default_color=colorant"green")))

# push!(lyrs, Gadfly.layer(y=rangesMM0_5, Geom.line, Theme(default_color=colorant"yellow")))
# push!(lyrs, Gadfly.layer(y=rangesMM0_MM5, Geom.line, Theme(default_color=colorant"red")))

push!(lyrs, Gadfly.layer(y=rangesMM0_10, Geom.point, Theme(default_color=colorant"magenta", default_point_size=ptsize)))
push!(lyrs, Gadfly.layer(y=rangesMM0_MM10, Geom.point, Theme(default_color=colorant"black", default_point_size=ptsize)))

# push!(lyrs, Gadfly.layer(y=rangesMM0_15, Geom.line, Theme(default_color=colorant"cyan")))
# push!(lyrs, Gadfly.layer(y=rangesMM0_MM15, Geom.line, Theme(default_color=colorant"red")))

push!(lyrs, Gadfly.layer(y=rangesMM0_20, Geom.point, Theme(default_color=colorant"blue", default_point_size=ptsize)))
push!(lyrs, Gadfly.layer(y=rangesMM0_MM20, Geom.point, Theme(default_color=colorant"black", default_point_size=ptsize)))

push!(lyrs, Coord.Cartesian(xmin=0,xmax=length(rangesMM0_0),
                            ymin=0,ymax=350))

push!(lyrs,
  Guide.xlabel("Individual poses")
)
push!(lyrs,
  Guide.ylabel("| X - X' |,  [meters]")
)
push!(lyrs,
Guide.manual_color_key("", ["MM-MAP", "MLE ~0%", "MLE 1%", "MLE 10%", "MLE 20%"], ["black", "deepskyblue", "green", "magenta", "blue"])
)
push!(lyrs,
Theme(key_position=:top)
)

lpl = Gadfly.plot(lyrs...)
draw(PDF("poseErrs.pdf",15cm,8cm),lpl)
lpl

pl = drawCompPosesLandm(MMMAP10,UMMLE10d, UMMLE10fg, lbls=false,drawunilm=false)
pl.coord = Coord.Cartesian(xmin=-120,xmax=250,ymin=-100,ymax=250)
draw(PNG("VicPrk10.png",15cm,12cm),pl)


vpl = vstack(pl,lpl);
draw(PDF("test.pdf",15cm,24cm),vpl)
