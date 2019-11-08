using Distributed
addprocs(4)
# using Revise
using GraphPlot
using Colors
using DistributedFactorGraphs
using RoME
@everywhere using RoME
using RoMEPlotting
Gadfly.set_default_plot_size(35cm,25cm)
# using Amphitheatre

import Base:*

*(a::Symbol, b::AbstractString)::Symbol = Symbol(string(a,b))

## First add the designed landmark positions
function addLandmarkTrueWithPrior!(dfg::AbstractDFG,
                                   key::Symbol,
                                   vals::Vector{<: Real},
                                   lgt_cov::Array{Float64,2}=0.01*Matrix(LinearAlgebra.I, 2,2)  )
  #
  if !hasVariable(dfg, key)
    addVariable!(dfg, key, Point2)
    pp = PriorPoint2(MvNormal(vals, lgt_cov))
    addFactor!(dfg, [key], pp)
  end
  nothing
end

function calcPosePointBearingRange(pose::Vector{<:Real},
                                   poin::Vector{<:Real}  )
  #
  @assert length(pose) == 3
  @assert length(poin) == 2

  #
  DD = poin-pose[1:2]
  ran = norm(DD)
  phi = atan(DD[2], DD[1])
  the = TU.wrapRad(phi - pose[3])

  #
  return the, ran
end


macro skipline(skip, operation)
  :($skip ? nothing : $operation)
end


# fg = LightDFG{NoSolverParams}()
# addVariable!(fg, DFGVariable(:x1))
# dfgplot(fg)

# ground truth [x,y,θ]
d2r(x) = x*pi/180

# ground = [[260, 720, d2r(0)],
#           [260, 720, d2r(-90)],
#           [260, 720, d2r(180)],
#           [260, 720, d2r(90)],
#           [310, 800, d2r(0)],
#           [310, 800, d2r(-90)],
#           [310, 800, d2r(180)],
#           [310, 800, d2r(90)]]

ground = Dict{Symbol, Vector{Float64}}(
  :x0=>[260.0, 720.0, 0.0],
  :x1=>[260.0, 720.0, -pi/2],
  :x2=>[260.0, 720.0, -pi],
  :x3=>[260.0, 720.0, pi/2],
  :x4=>[260.0, 800.0, 0.0],
  :x5=>[310.0, 800.0, 0.0],
  :x6=>[310.0, 800.0, -pi/2],
  :x7=>[310.0, 800.0, -pi],
  :x8=>[310.0, 800.0, pi/2],
)

## Add all variables as design truth (for use as multihypos later)
# Ground truth for landmarks (designed)
landmarks_real = Dict(
    :l1=>[370,780],
    :l2=>[380,780],
    :l3=>[380,670],
    :l4=>[370,670],
    :l5=>[190,670],
    :l6=>[170,670],
    :l7=>[170,830],
    :l8=>[370,790],
    :l9=>[280,830],
    :l10=>[440,790],
    :l11=>[430,790],
    :l12=>[400,920])

# map with Dx=-10 on l1-l4 (ground truth from measured, varies from design)
landmarks_design = Dict(
    :l1=>[360,780],
    :l2=>[370,780],
    :l3=>[370,670],
    :l4=>[360,670],
    :l5=>[190,670],
    :l6=>[170,670],
    :l7=>[170,830],
    :l8=>[360,790],
    :l9=>[280,830],
    :l10=>[440,790],
    :l11=>[430,790],
    :l12=>[400,920])


## parameters
autoinit = true
pp_σx = 0.1
pp_σy = 0.1
pp_σψ = 0.01
br_σb = 0.03
br_σρ = 0.3


SKL = false

## Start the factor graph construction
fg = LightDFG{SolverParams}(params=SolverParams())

## develop the robotic factor graph elements
# x0
x0_prior = [ground[:x0],[0.1,0.1,0.01]] #fixed
addVariable!(fg, :x0, Pose2)
addFactor!(fg, [:x0],  PriorPose2( MvNormal(x0_prior[1], Matrix(Diagonal(x0_prior[2].^2))) ), autoinit=autoinit )

    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l1])
    # x0l1_br = [[d2r(27.65), 0.02], [118.53, 0.2]]#(tagψ, σtagψ), Normal(tagrange, σtagrange)
    x0l1_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    @skipline SKL addVariable!(fg, :l1, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l1_0, landmarks_design[:l1])
    @skipline SKL addFactor!(fg, [:x0, :l1, :l1_0], Pose2Point2BearingRange(Normal(x0l1_br[1]...), Normal(x0l1_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])


    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l2])
    x0l2_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x0l2_br = [[d2r(25.56), 0.02], [127.48, 0.2]]
    @skipline SKL addVariable!(fg, :l2, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l2_0, landmarks_design[:l2])
    @skipline SKL addFactor!(fg, [:x0, :l2, :l2_0], Pose2Point2BearingRange(Normal(x0l2_br[1]...), Normal(x0l2_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l3])
    x0l3_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x0l3_br = [[d2r(-25.56), 0.02], [127.48, 0.2]]
    @skipline SKL addVariable!(fg, :l3, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l3_0, landmarks_design[:l3])
    @skipline SKL addFactor!(fg, [:x0, :l3, :l3_0], Pose2Point2BearingRange(Normal(x0l3_br[1]...), Normal(x0l3_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l4])
    x0l4_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x0l4_br = [[d2r(-27.65), 0.02], [118.53, 0.2]]
    @skipline SKL addVariable!(fg, :l4, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l4_0, landmarks_design[:l4])
    @skipline SKL addFactor!(fg, [:x0, :l4, :l4_0], Pose2Point2BearingRange(Normal(x0l4_br[1]...), Normal(x0l4_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

## test for multimodality
# dfgplot(fg)
# drawGraph(fg)
# # ensureAllInitialized!(fg)
getSolverParams(fg).drawtree=true
getSolverParams(fg).showtree=true
# getSolverParams(fg).dbg=true
# getSolverParams(fg).async=false
tree, smt, hist = solveTree!(fg, recordcliqs=ls(fg));
drawPosesLandms(fg, drawhist=true)



# x1
x0x1_pp = [[0,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x1, Pose2)
addFactor!(fg, [:x0, :x1], Pose2Pose2(MvNormal(x0x1_pp[1], Matrix(Diagonal(x0x1_pp[2].^2)))), autoinit=autoinit)
# ensureAllInitialized!(fg)
# x2
x1x2_pp = [[0,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x2, Pose2)
addFactor!(fg, [:x1, :x2], Pose2Pose2(MvNormal(x1x2_pp[1], Matrix(Diagonal(x1x2_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x2], landmarks_real[:l5])
    x2l5_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x2l5_br = [[d2r(-143.75+180), 0.1], [92.65, 1]]
    @skipline SKL addVariable!(fg, :l5, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l5_0, landmarks_design[:l5])
    @skipline SKL addFactor!(fg, [:x2, :l5, :l5_0], Pose2Point2BearingRange(Normal(x2l5_br[1]...), Normal(x2l5_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

    br_calc = calcPosePointBearingRange(ground[:x2], landmarks_real[:l6])
    x2l6_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x2l6_br = [[d2r(-149.93+180), 0.1], [109.77, 1]]
    @skipline SKL addVariable!(fg, :l6, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l6_0, landmarks_design[:l6])
    @skipline SKL addFactor!(fg, [:x2, :l6, :l6_0], Pose2Point2BearingRange(Normal(x2l6_br[1]...), Normal(x2l6_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

# ensureAllInitialized!(fg)
# x3:
x2x3_pp = [[0,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x3, Pose2)
addFactor!(fg, [:x2, :x3], Pose2Pose2(MvNormal(x2x3_pp[1], Matrix(Diagonal(x2x3_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x3], landmarks_real[:l7])
    x3l7_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x3l7_br = [[d2r(132.4-90), 0.1], [141.32, 1]]
    @skipline SKL addVariable!(fg, :l7, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l7_0, landmarks_design[:l7])
    @skipline SKL addFactor!(fg, [:x3, :l7, :l7_0], Pose2Point2BearingRange(Normal(x3l7_br[1]...), Normal(x3l7_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

# ensureAllInitialized!(fg)
# x4 drive + 80
x3x4_pp = [[80,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x4, Pose2)
addFactor!(fg, [:x3, :x4], Pose2Pose2(MvNormal(x3x4_pp[1], Matrix(Diagonal(x3x4_pp[2].^2)))), autoinit=autoinit)

# ensureAllInitialized!(fg)
# x5 turn -90 -> drive + 50
x4x5_pp = [[50,0,d2r(0)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x5, Pose2)
addFactor!(fg, [:x4, :x5], Pose2Pose2(MvNormal(x4x5_pp[1], Matrix(Diagonal(x4x5_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x5], landmarks_real[:l8])
    x5l8_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x5l8_br = [[d2r(-15.26), 0.1], [57.01, 1]]
    @skipline SKL addVariable!(fg, :l8, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l8_0, landmarks_design[:l8])
    @skipline SKL addFactor!(fg, [:x5, :l8, :l8_0], Pose2Point2BearingRange(Normal(x5l8_br[1]...), Normal(x5l8_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.5;0.5])

    br_calc = calcPosePointBearingRange(ground[:x5], landmarks_real[:l10])
    x5l10_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x5l10_br = [[d2r(-6.84), 0.1], [105.84, 1]]
    @skipline SKL addVariable!(fg, :l10, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l10_0, landmarks_design[:l10])
    @skipline SKL addFactor!(fg, [:x5, :l10, :l10_0], Pose2Point2BearingRange(Normal(x5l10_br[1]...), Normal(x5l10_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

    br_calc = calcPosePointBearingRange(ground[:x5], landmarks_real[:l11])
    x5l11_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x5l11_br = [[d2r(-7.43), 0.1], [113.59, 1]]
    @skipline SKL addVariable!(fg, :l11, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l11_0, landmarks_design[:l11])
    @skipline SKL addFactor!(fg, [:x5, :l11, :l11_0], Pose2Point2BearingRange(Normal(x5l11_br[1]...), Normal(x5l11_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])


#
# drawGraph(fg)
# ensureAllInitialized!(fg)
# drawPosesLandms(fg)
getSolverParams(fg).drawtree=true
getSolverParams(fg).showtree=true
tree, smt, hist = solveTree!(fg);

drawPosesLandms(fg, drawhist=true, contour=false)
drawLandms(fg, drawhist=true, contour=false, from=1, to=4)
reportFactors(fg, Pose2Pose2)
#


# ensureAllInitialized!(fg)
# x6
x5x6_pp = [[0,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x6, Pose2)
addFactor!(fg, [:x5, :x6], Pose2Pose2(MvNormal(x5x6_pp[1], Matrix(Diagonal(x5x6_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x6], landmarks_real[:l3])
    x6l3_br  = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x6l3_br = [[d2r(-64.29+90), 0.1], [145.07, 1]]
    # addVariable!(fg, :l3, Point2)
    @skipline SKL addFactor!(fg, [:x6, :l3, :l3_0], Pose2Point2BearingRange(Normal(x6l3_br[1]...), Normal(x6l3_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])

    br_calc = calcPosePointBearingRange(ground[:x6], landmarks_real[:l4])
    x6l4_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x6l4_br = [[d2r(-67.83+90), 0.1], [141.01, 1]]
    # addVariable!(fg, :l4, Point2)
    @skipline SKL addFactor!(fg, [:x6, :l4, :l4_0], Pose2Point2BearingRange(Normal(x6l4_br[1]...), Normal(x6l4_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])


## debug check solve here
# drawGraph(fg)

# ensureAllInitialized!(fg)
# drawPosesLandms(fg)
# getSolverParams(fg).drawtree=true
# getSolverParams(fg).showtree=true
# tree, smt, hist = solveTree!(fg);
tree, smt, hist = solveTree!(fg, tree);
drawPosesLandms(fg)

## end debugging block


# x7
x6x7_pp = [[0,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x7, Pose2)
addFactor!(fg, [:x6, :x7], Pose2Pose2(MvNormal(x6x7_pp[1], Matrix(Diagonal(x6x7_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x7], landmarks_real[:l7])
    x7l7_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x7l7_br = [[d2r(137-180), 0.1], [147.89, 1]]
    pp_x7l7_br = Pose2Point2BearingRange(Normal(x7l7_br[1]...), Normal(x7l7_br[2]...))
    @skipline SKL addFactor!(fg, [:x7; :l7; :l7_0], pp_x7l7_br, autoinit=autoinit, multihypo=[1.0;0.7;0.3])

    br_calc = calcPosePointBearingRange(ground[:x7], landmarks_real[:l9])
    x7l9_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x7l9_br = [[d2r(144.46-180), 0.1], [42.04, 1]]
    @skipline SKL addVariable!(fg, :l9, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l9_0, landmarks_design[:l9])
    @skipline SKL addFactor!(fg, [:x7, :l9, :l9_0], Pose2Point2BearingRange(Normal(x7l9_br[1]...), Normal(x7l9_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])


# drawGraph(fg)
# pts = approxConv(fg, :x7l7l7bf1, :l7)
# pts = approxConv(fg, :x7l7l7bf1, :l7b)


# ensureAllInitialized!(fg)
# x8
x7x8_pp = [[0,0,d2r(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x8, Pose2)
addFactor!(fg, [:x7, :x8], Pose2Pose2(MvNormal(x7x8_pp[1], Matrix(Diagonal(x7x8_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x8], landmarks_real[:l12])
    x8l12_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x8l12_br = [[d2r(53.53-90), 0.1], [133.32, 1]]
    @skipline SKL addVariable!(fg, :l12, Point2)
    @skipline SKL addLandmarkTrueWithPrior!(fg, :l12_0, landmarks_design[:l12])
    @skipline SKL addFactor!(fg, [:x8, :l12, :l12_0], Pose2Point2BearingRange(Normal(x8l12_br[1]...), Normal(x8l12_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.7;0.3])



# ensureAllInitialized!(fg)

##
p = DFGPlotProps(  (var=colorant"seagreen", fac=colorant"cyan3"),
                            (var=1.0, fac=0.3),
                            (var=:box, fac=:elipse),
                            spring_layout,#spectral_layout
                            true)
dfgplot(fg,p)


## 3D VIS ==================

fgmax = deepcopy(fg)
fgmax.robotId = "Max"

fgvis = BasicFactorGraphPose(fg, meanmax=:mean)
fgvismax = BasicFactorGraphPose(fgmax, meanmax=:max)

visdatasets = AbstractAmphitheatre[fgvis]
push!(visdatasets, fgvismax)
vis, vistask = visualize(visdatasets, start_browser=true)

stopAmphiVis!()
vistask = Amphitheatre.restartVisualize(vis, visdatasets)

## END VIS 3D



# getSolverParams(fg).async = false
# tree, smt, hist = solveTree!(fg, tree);
tree, smt, hist = solveTree!(fg);


## show the delta

pl = drawPosesLandms(fg)
pl |> PDF("/tmp/caesar/random/mapped4.pdf")

pl = drawLandms(fg, drawhist=true, to=4)
pl |> PDF("/tmp/caesar/random/badwall4.pdf")


pts = getVariable(fg, :l8) |> getKDE |> getPoints
pl = Gadfly.plot(x=pts[1,:],y=pts[2,:],Geom.hexbin)
pl |> PDF("/tmp/caesar/random/bad_l8.pdf")


#

plotKDE(fg, :l3)

landmarks_design[:l3]
landmarks_real[:l3]





##

syms = sortDFG(ls(fg, r"x"))
ppes = Base.map((x)->getVariablePPE(fg,x), syms)
trivial_poses = Dict(syms .=> ppes)

# syms = sortDFG(ls(fg, r"l\d*_"))
# ppes = Base.map((x)->getVariablePPE(fg,x), syms)
# design_landms = Dict(syms .=> ppes)
#
# syms = sortDFG(setdiff(ls(fg, r"l\d*"),ls(fg, r"l\d*_")) )
# ppes = Base.map((x)->getVariablePPE(fg,x), syms)
# design_mapped = Dict(syms .=> ppes)
#

# ## Round out poses for convenience (getting ground truth)
# for sy in syms
#     trivial_poses[sy][1:2] .= round.(trivial_poses[sy][1:2])
#     trivial_poses[sy][3] = round.(trivial_poses[sy][3], digits=1)
#     println(sy, "=>", trivial_poses[sy])
# end
0


## DRAW THE MAP





## END DRAW


# getVariableSummary(var::DFGVariable, softt::Pose2) = getPPE(var, softt)
#
# getVariableSummary(var::DFGVariable) = getVariableSummary(var::DFGVariable, getSofttype(var))
#
# function getVariableSummary(dfg::AbstractDFG, sym::Symbol)
#
# var = getVariableSummary(getVariable(dfg, sym))
#
# end



##

# using RoMEPlotting

plotPose(fg, [:x0])
plotPose(fg, [:x1])
plotPose(fg, [:x2])
plotPose(fg, [:x3])
plotPose(fg, [:x0,:x1,:x2,:x3], levels=1)

## ##
# x?
# x_x?_pp = [[0,0,d2r(-90)], [0.1, 0.1, 0.01]]
# x?l__br = [[d2r(_), 0.1], [_, 1]]
# x?l__br = [[d2r(_), 0.1], [_, 1]]


##

plotLocalProduct(fg, :x0)
plotLocalProduct(fg, :x1)
plotLocalProduct(fg, :x2)

plotLocalProduct(fg, :l7, levels=3)


plotLocalProduct(fg, :l1_0)
plotLocalProduct(fg, :l2_0)
plotLocalProduct(fg, :l3_0)
plotLocalProduct(fg, :l4_0)

plotLocalProduct(fg, :l4)

ls(fg, :l7)

##

##

drawPosesLandms(fg)

reportFactors(fg, Pose2Point2BearingRange);
reportFactors(fg, Pose2Pose2);


plotPose(fg, :x7)
plotKDE(fg, [:l7;:l7_1])

## batch resolve

tree, smt, hist = solveTree!(fg);

##











# ensureAllInitialized!(fg)
# cu, pr = predictVariableByFactor(fg, :l7, pp_x7l7_br, [:x7;:l7])
# if minkld(cu, pr) < 3
    # addFactor!(fg, [:x7, :l7], pp_x7l7_br, autoinit=autoinit)
# else
#     addVariable!(fg, :l7_1, Point2)
# end



## debug multimodality

# check fake BR factor
tfg = initfg()
addVariable!(tfg, :x0, Pose2)
addVariable!(tfg, :l1, Point2)
manualinit!(tfg, :x0, getKDE(fg, :x0))
manualinit!(tfg, :l1, getKDE(fg, :l1))
pp = Pose2Point2BearingRange(Normal(x0l1_br[1]...), Normal(x0l1_br[2]...))
addFactor!(tfg, [:x0;:l1],pp)
plotFactor(tfg, :x0l1f1)


plotLocalProduct(fg, :x0)

ls(fg, :l1)

pl = plotLocalProduct(fg, :l1, levels=3)
plotLocalProduct(fg, :l1_0)
plotLocalProduct(fg, :l1, levels=10)
plotLocalProduct(fg, :l2_0)
plotLocalProduct(fg, :l2)
plotLocalProduct(fg, :l3_0)
plotLocalProduct(fg, :l3)
plotLocalProduct(fg, :l4_0)
plotLocalProduct(fg, :l4)

landmarks_design[:l1]
landmarks_real[:l1]

pts = approxConv(fg, :x0l1l1_0f1, :l1)

pts = approxConv(fg, :x0l1l1_0f1, :l1_0)


pl2 = Gadfly.plot(x=pts[1,:],y=pts[2,:], Geom.hexbin)
union!(pl2.layers, pl.layers)
pl2
pl |> PDF("/tmp/caesar/random/test.pdf")


pts = approxConv(fg, :l1_0f1, :l1_0)


drawLandms(fg, drawhist=true)

pts = getPoints(getKDE(fg, :l1))
plotKDE(fg, :l1, levels=25)


##
