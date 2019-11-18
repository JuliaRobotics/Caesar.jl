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


function getCoordCartesianFromKDERange(dfg::AbstractDFG, varsym::Symbol; digits::Int=-2, extend::Float64=0.5)
    ax = round.(getKDERange(getKDE(dfg, varsym), extend=extend), digits=digits)
    if size(ax, 1) == 1
      return Coord.Cartesian(xmin=ax[1,1], xmax=ax[1,2])
    elseif size(ax, 1) == 2
      return Coord.Cartesian(xmin=ax[1,1], xmax=ax[1,2],ymin=ax[2,1], ymax=ax[2,2])
    else
      error("only do 1D or 2D")
    end
end
# import KernelDensityEstimate: getCoordCartesianFromKDERange


function addMaptoPlot!(pl)
    # plot the design coordinates
    map_coords = Array{Array{Float64,1},1}[[[0.0, 0.0], [0.0, 330.0], [40.0, 330.0], [40.0, 310.0], [20.0, 310.0], [20.0, 150.0], [130.0, 150.0], [130.0, 140.0], [20.0, 140.0], [20.0, 20.0], [250.0, 20.0], [250.0, 60.0], [260.0, 60.0], [260.0, 20.0], [290.0, 20.0], [290.0, 60.0], [300.0, 60.0], [300.0, 20.0], [430.0, 20.0], [430.0, 110.0], [300.0, 110.0], [300.0, 100.0], [290.0, 100.0], [290.0, 190.0], [280.0, 190.0], [280.0, 200.0], [320.0, 200.0], [320.0, 190.0], [300.0, 190.0], [300.0, 120.0], [430.0, 120.0], [430.0, 190.0], [410.0, 190.0], [410.0, 200.0], [430.0, 200.0], [430.0, 310.0], [390.0, 310.0], [390.0, 330.0], [450.0, 330.0], [450.0, 0.0], [0.0, 0.0]], [[90.0, 310.0], [90.0, 330.0], [170.0, 330.0], [170.0, 310.0], [90.0, 310.0]], [[230.0, 310.0], [340.0, 310.0], [340.0, 330.0], [210.0, 330.0], [210.0, 310.0], [220.0, 310.0], [220.0, 200.0], [210.0, 200.0], [210.0, 190.0], [240.0, 190.0], [240.0, 200.0], [230.0, 200.0], [230.0, 310.0]]]

    for m_coord in map_coords
        Xpp = map(p->p[1] + 150, m_coord)
        Ypp = map(p->-p[2] +330 + 650, m_coord)

        push!(pl.layers, Gadfly.layer(x=Xpp,y=Ypp,Geom.path(), Theme(line_width=1pt))[1])
    end

    # plot the real map variation
    built_coords = Array{Array{Float64,1},1}[[[230.0, 310.0], [340.0, 310.0], [340.0, 330.0], [210.0, 330.0], [210.0, 310.0], [220.0, 310.0], [220.0, 200.0], [210.0, 200.0], [210.0, 190.0], [240.0, 190.0], [240.0, 200.0], [230.0, 200.0], [230.0, 310.0]]]

    for b_coord in built_coords
        Xpp = map(p->p[1] + 150+30, b_coord)
        Ypp = map(p->-p[2] +330 + 650+20, b_coord)

        push!(pl.layers, Gadfly.layer(x=Xpp,y=Ypp,Geom.path(), Theme(line_width=1pt, default_color=colorant"red"))[1])
    end

    return pl
end

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


macro doline(skip, operation)
  :($skip ? $operation : nothing)
end


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
    :l1=>[360+30,780+20],
    :l2=>[370+30,780+20],
    :l3=>[370+30,670+20],
    :l4=>[360+30,670+20],
    :l5=>[360+30,790+20],
    :l8=>[190,670],
    :l6=>[170,670],
    :l7=>[170,830],
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
    :l5=>[360,790],
    :l8=>[190,670],
    :l6=>[170,670],
    :l7=>[170,830],
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
pMeas, pDesg = 0.7, 0.3

SKL = true

## Start the factor graph construction
fg = LightDFG{SolverParams}(params=SolverParams())

## develop the robotic factor graph elements
# x0
x0_prior = [ground[:x0],[0.1,0.1,0.01]] #fixed
addVariable!(fg, :x0, Pose2)
addFactor!(fg, [:x0],  PriorPose2( MvNormal(x0_prior[1], Matrix(Diagonal(x0_prior[2].^2))) ), autoinit=autoinit )

    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l1])
    # x0l1_br = [[deg2rad(27.65), 0.02], [118.53, 0.2]]#(tagψ, σtagψ), Normal(tagrange, σtagrange)
    x0l1_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    @doline SKL addVariable!(fg, :l1, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l1_0, landmarks_design[:l1])
    @doline SKL addFactor!(fg, [:x0, :l1, :l1_0], Pose2Point2BearingRange(Normal(x0l1_br[1]...), Normal(x0l1_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])


    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l2])
    x0l2_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x0l2_br = [[deg2rad(25.56), 0.02], [127.48, 0.2]]
    @doline SKL addVariable!(fg, :l2, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l2_0, landmarks_design[:l2])
    @doline SKL addFactor!(fg, [:x0, :l2, :l2_0], Pose2Point2BearingRange(Normal(x0l2_br[1]...), Normal(x0l2_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l3])
    x0l3_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x0l3_br = [[deg2rad(-25.56), 0.02], [127.48, 0.2]]
    @doline SKL addVariable!(fg, :l3, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l3_0, landmarks_design[:l3])
    @doline SKL addFactor!(fg, [:x0, :l3, :l3_0], Pose2Point2BearingRange(Normal(x0l3_br[1]...), Normal(x0l3_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

    br_calc = calcPosePointBearingRange(ground[:x0], landmarks_real[:l4])
    x0l4_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x0l4_br = [[deg2rad(-27.65), 0.02], [118.53, 0.2]]
    @doline SKL addVariable!(fg, :l4, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l4_0, landmarks_design[:l4])
    @doline SKL addFactor!(fg, [:x0, :l4, :l4_0], Pose2Point2BearingRange(Normal(x0l4_br[1]...), Normal(x0l4_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

## test for multimodality
# dfgplot(fg)
# drawGraph(fg)
# # ensureAllInitialized!(fg)
getSolverParams(fg).drawtree=true
getSolverParams(fg).showtree=true
# getSolverParams(fg).dbg=true
# getSolverParams(fg).async=false
tree, smt, hist = solveTree!(fg);
pl = drawPosesLandms(fg, drawhist=true, contour=false)
addMaptoPlot!(pl)
pl.coord = Coord.Cartesian(xmin=100, xmax=450, ymin=600, ymax=850)
pl
# pl |> PDF(ENV["HOME"]*"/Downloads/test.pdf")

# plot l1
pl1 = drawLandms(fg, to=1, drawhist=true, contour=false)
addMaptoPlot!(pl1)
pl1.coord = getCoordCartesianFromKDERange(fg, :l1, digits=0)
pl1

# plot l2
pl2 = drawLandms(fg, from=2,to=2, drawhist=true, contour=false)
addMaptoPlot!(pl2)
pl2.coord = getCoordCartesianFromKDERange(fg, :l2, digits=0)
pl2

# plot l3
pl3 = drawLandms(fg, from=3,to=3, drawhist=true, contour=false)
addMaptoPlot!(pl3)
pl3.coord = getCoordCartesianFromKDERange(fg, :l3, digits=0)
pl3

# plot l4
pl4 = drawLandms(fg, from=4,to=4, drawhist=true, contour=false)
addMaptoPlot!(pl4)
pl4.coord = getCoordCartesianFromKDERange(fg, :l4, digits=0)
pl4

pl = vstack(hstack(pl1,pl2), hstack(pl3,pl4))
# pl |> PDF(ENV["HOME"]*"/Downloads/stack.pdf")


# x1
x0x1_pp = [[0,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x1, Pose2)
addFactor!(fg, [:x0, :x1], Pose2Pose2(MvNormal(x0x1_pp[1], Matrix(Diagonal(x0x1_pp[2].^2)))), autoinit=autoinit)
# ensureAllInitialized!(fg)
# x2
x1x2_pp = [[0,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x2, Pose2)
addFactor!(fg, [:x1, :x2], Pose2Pose2(MvNormal(x1x2_pp[1], Matrix(Diagonal(x1x2_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x2], landmarks_real[:l5])
    x2l5_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x2l5_br = [[deg2rad(-143.75+180), 0.1], [92.65, 1]]
    @doline SKL addVariable!(fg, :l5, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l5_0, landmarks_design[:l5])
    @doline SKL addFactor!(fg, [:x2, :l5, :l5_0], Pose2Point2BearingRange(Normal(x2l5_br[1]...), Normal(x2l5_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

    br_calc = calcPosePointBearingRange(ground[:x2], landmarks_real[:l6])
    x2l6_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x2l6_br = [[deg2rad(-149.93+180), 0.1], [109.77, 1]]
    @doline SKL addVariable!(fg, :l6, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l6_0, landmarks_design[:l6])
    @doline SKL addFactor!(fg, [:x2, :l6, :l6_0], Pose2Point2BearingRange(Normal(x2l6_br[1]...), Normal(x2l6_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

# ensureAllInitialized!(fg)
# x3:
x2x3_pp = [[0,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x3, Pose2)
addFactor!(fg, [:x2, :x3], Pose2Pose2(MvNormal(x2x3_pp[1], Matrix(Diagonal(x2x3_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x3], landmarks_real[:l7])
    x3l7_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x3l7_br = [[deg2rad(132.4-90), 0.1], [141.32, 1]]
    @doline SKL addVariable!(fg, :l7, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l7_0, landmarks_design[:l7])
    @doline SKL addFactor!(fg, [:x3, :l7, :l7_0], Pose2Point2BearingRange(Normal(x3l7_br[1]...), Normal(x3l7_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

# ensureAllInitialized!(fg)
# x4 drive + 80
x3x4_pp = [[80,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x4, Pose2)
addFactor!(fg, [:x3, :x4], Pose2Pose2(MvNormal(x3x4_pp[1], Matrix(Diagonal(x3x4_pp[2].^2)))), autoinit=autoinit)

# ensureAllInitialized!(fg)
# x5 turn -90 -> drive + 50
x4x5_pp = [[50,0,deg2rad(0)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x5, Pose2)
addFactor!(fg, [:x4, :x5], Pose2Pose2(MvNormal(x4x5_pp[1], Matrix(Diagonal(x4x5_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x5], landmarks_real[:l8])
    x5l8_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x5l8_br = [[deg2rad(-15.26), 0.1], [57.01, 1]]
    @doline SKL addVariable!(fg, :l8, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l8_0, landmarks_design[:l8])
    @doline SKL addFactor!(fg, [:x5, :l8, :l8_0], Pose2Point2BearingRange(Normal(x5l8_br[1]...), Normal(x5l8_br[2]...)), autoinit=autoinit, multihypo=[1.0;0.5;0.5])

    br_calc = calcPosePointBearingRange(ground[:x5], landmarks_real[:l10])
    x5l10_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x5l10_br = [[deg2rad(-6.84), 0.1], [105.84, 1]]
    @doline SKL addVariable!(fg, :l10, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l10_0, landmarks_design[:l10])
    @doline SKL addFactor!(fg, [:x5, :l10, :l10_0], Pose2Point2BearingRange(Normal(x5l10_br[1]...), Normal(x5l10_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

    br_calc = calcPosePointBearingRange(ground[:x5], landmarks_real[:l11])
    x5l11_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x5l11_br = [[deg2rad(-7.43), 0.1], [113.59, 1]]
    @doline SKL addVariable!(fg, :l11, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l11_0, landmarks_design[:l11])
    @doline SKL addFactor!(fg, [:x5, :l11, :l11_0], Pose2Point2BearingRange(Normal(x5l11_br[1]...), Normal(x5l11_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])


#
drawGraph(fg)
# ensureAllInitialized!(fg)
# drawPosesLandms(fg)
# getSolverParams(fg).drawtree=true
# getSolverParams(fg).showtree=true
tree, smt, hist = solveTree!(fg);

pl = drawPosesLandms(fg, drawhist=true, contour=false)
addMaptoPlot!(pl)
pl.coord = Coord.Cartesian(xmin=100, xmax=500, ymin=600, ymax=900)
pl
# pl |> PDF(ENV["HOME"]*"/Downloads/explore1.pdf")



pl5 = drawLandms(fg, from=5,to=5, drawhist=true, contour=false)
addMaptoPlot!(pl5)
pl5.coord = getCoordCartesianFromKDERange(fg, :l5, digits=0)
pl5


# reportFactors(fg, Pose2Pose2)
#

# drawLandms(fg, to=1, drawhist=true, contour=false)

# plot the bimodal wall
plotVariable2D(fg, :l1, refs=[landmarks_design, landmarks_real])
plotVariable2D(fg, :l2, refs=[landmarks_design, landmarks_real])
plotVariable2D(fg, :l3, refs=[landmarks_design, landmarks_real])
plotVariable2D(fg, :l4, refs=[landmarks_design, landmarks_real])
plotVariable2D(fg, :l5, refs=[landmarks_design, landmarks_real])
# plot unimodals
plotVariable2D(fg, :l6, refs=[landmarks_design, landmarks_real])
plotVariable2D(fg, :l7, refs=[landmarks_design, landmarks_real])
plotVariable2D(fg, :l8, refs=[landmarks_design, landmarks_real])

reportFactors(fg, Pose2Pose2)

# plotLocalProduct(fg, :x4, levels=5)

# ensureAllInitialized!(fg)
# x6
x5x6_pp = [[0,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x6, Pose2)
addFactor!(fg, [:x5, :x6], Pose2Pose2(MvNormal(x5x6_pp[1], Matrix(Diagonal(x5x6_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x6], landmarks_real[:l3])
    x6l3_br  = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x6l3_br = [[deg2rad(-64.29+90), 0.1], [145.07, 1]]
    # addVariable!(fg, :l3, Point2)
    @doline SKL addFactor!(fg, [:x6, :l3, :l3_0], Pose2Point2BearingRange(Normal(x6l3_br[1]...), Normal(x6l3_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

    br_calc = calcPosePointBearingRange(ground[:x6], landmarks_real[:l4])
    x6l4_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x6l4_br = [[deg2rad(-67.83+90), 0.1], [141.01, 1]]
    # addVariable!(fg, :l4, Point2)
    @doline SKL addFactor!(fg, [:x6, :l4, :l4_0], Pose2Point2BearingRange(Normal(x6l4_br[1]...), Normal(x6l4_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])


## debug check solve here
# drawGraph(fg)

# ensureAllInitialized!(fg)
# drawPosesLandms(fg)
# getSolverParams(fg).drawtree=true
# getSolverParams(fg).showtree=true
tree, smt, hist = solveTree!(fg);
# tree, smt, hist = solveTree!(fg, tree);
pl = drawPosesLandms(fg, drawhist=true, contour=false)
addMaptoPlot!(pl)
# pl |> PDF(ENV["HOME"]*"/Downloads/explore2.pdf")



# x7
x6x7_pp = [[0,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x7, Pose2)
addFactor!(fg, [:x6, :x7], Pose2Pose2(MvNormal(x6x7_pp[1], Matrix(Diagonal(x6x7_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x7], landmarks_real[:l7])
    x7l7_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x7l7_br = [[deg2rad(137-180), 0.1], [147.89, 1]]
    pp_x7l7_br = Pose2Point2BearingRange(Normal(x7l7_br[1]...), Normal(x7l7_br[2]...))
    @doline SKL addFactor!(fg, [:x7; :l7; :l7_0], pp_x7l7_br, autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])

    br_calc = calcPosePointBearingRange(ground[:x7], landmarks_real[:l9])
    x7l9_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x7l9_br = [[deg2rad(144.46-180), 0.1], [42.04, 1]]
    @doline SKL addVariable!(fg, :l9, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l9_0, landmarks_design[:l9])
    @doline SKL addFactor!(fg, [:x7, :l9, :l9_0], Pose2Point2BearingRange(Normal(x7l9_br[1]...), Normal(x7l9_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])


# drawGraph(fg)
# pts = approxConv(fg, :x7l7l7bf1, :l7)
# pts = approxConv(fg, :x7l7l7bf1, :l7b)


# ensureAllInitialized!(fg)
# x8
x7x8_pp = [[0,0,deg2rad(-90)], [pp_σx, pp_σy, pp_σψ]]
addVariable!(fg, :x8, Pose2)
addFactor!(fg, [:x7, :x8], Pose2Pose2(MvNormal(x7x8_pp[1], Matrix(Diagonal(x7x8_pp[2].^2)))), autoinit=autoinit)

    br_calc = calcPosePointBearingRange(ground[:x8], landmarks_real[:l12])
    x8l12_br = [[br_calc[1], br_σb], [br_calc[2], br_σρ]]
    # x8l12_br = [[deg2rad(53.53-90), 0.1], [133.32, 1]]
    @doline SKL addVariable!(fg, :l12, Point2)
    @doline SKL addLandmarkTrueWithPrior!(fg, :l12_0, landmarks_design[:l12])
    @doline SKL addFactor!(fg, [:x8, :l12, :l12_0], Pose2Point2BearingRange(Normal(x8l12_br[1]...), Normal(x8l12_br[2]...)), autoinit=autoinit, multihypo=[1.0;pMeas;pDesg])



# ensureAllInitialized!(fg)



# getSolverParams(fg).async = false
# tree, smt, hist = solveTree!(fg, tree);
tree, smt, hist = solveTree!(fg);


pl = drawPosesLandms(fg, drawhist=true, contour=false)
addMaptoPlot!(pl)
# pl |> PDF(ENV["HOME"]*"/Downloads/mapped.pdf")

pl = drawLandms(fg, drawhist=true, to=5, contour=false)
addMaptoPlot!(pl)
pl.coord = Coord.Cartesian(xmin=100, xmax=450, ymin=600, ymax=850)
pl
# pl |> PDF(ENV["HOME"]*"/Downloads/badwall.pdf")


drawGraph(fg)

# reportFactors(fg, Pose2Pose2);



0
## =============================================================================
## Deeper look at various other aspects.

# pl = plotVariable2D(fg, :l10)
#
# pl = drawLandms(fg, from=10, to=11, contour=false, drawhist=true)
# addMaptoPlot!(pl)
#
# pl.coord = getCoordCartesianFromKDERange(fg, :l10, digits=0, extend=10.0)
# pl


# ## quick check
#
# pl = drawLandms(fg, from=4, to=4, drawhist=true, contour=false)
# plotKDE(fg, :l4, levels=20)
# plotKDE(fg, :l4_0, levels=20)
# plotKDE(fg, [:l4;:l4_0])

##

#
# ## 3D VIS ==================
#
# using Amphitheatre
#
# fgmax = deepcopy(fg)
# fgmax.robotId = "Max"
#
# fgvis = BasicFactorGraphPose(fg, meanmax=:mean)
# fgvismax = BasicFactorGraphPose(fgmax, meanmax=:max)
#
# visdatasets = AbstractAmphitheatre[fgvis]
# push!(visdatasets, fgvismax)
# vis, vistask = visualize(visdatasets, start_browser=true)
#
# stopAmphiVis!()
# vistask = Amphitheatre.restartVisualize(vis, visdatasets)
#
# ## END VIS 3D





# plotVariable2D(fg, :l9, refs=[landmarks_design, landmarks_real])
# plotVariable2D(fg, :l10, refs=[landmarks_design, landmarks_real])
# plotVariable2D(fg, :l11, refs=[landmarks_design, landmarks_real])
# plotVariable2D(fg, :l12, refs=[landmarks_design, landmarks_real])
#
# plotLocalProduct(fg, :l3, levels=10)
# plotLocalProduct(fg, :l4, levels=10)
#
# # plotLocalProduct(fg, :x6, levels=10)
#
#
#
# syms = sortDFG(ls(fg, r"x"))
# ppes = Base.map((x)->calcVariablePPE(fg,x), syms)
# trivial_poses = Dict(syms .=> ppes)
#
# # syms = sortDFG(ls(fg, r"l\d*_"))
# # ppes = Base.map((x)->calcVariablePPE(fg,x), syms)
# # design_landms = Dict(syms .=> ppes)
# #
# # syms = sortDFG(setdiff(ls(fg, r"l\d*"),ls(fg, r"l\d*_")) )
# # ppes = Base.map((x)->calcVariablePPE(fg,x), syms)
# # design_mapped = Dict(syms .=> ppes)
# #
#
#
#
#
# ##
# p = DFGPlotProps(  (var=colorant"seagreen", fac=colorant"cyan3"),
#                             (var=1.0, fac=0.3),
#                             (var=:box, fac=:elipse),
#                             spring_layout,#spectral_layout
#                             true)
# dfgplot(fg,p)
#
#
#
# ##
#
# # using RoMEPlotting
#
# plotPose(fg, [:x0])
# plotPose(fg, [:x1])
# plotPose(fg, [:x2])
# plotPose(fg, [:x3])
# plotPose(fg, [:x0,:x1,:x2,:x3], levels=1)
#
#
# ##
#
# plotLocalProduct(fg, :x0)
# plotLocalProduct(fg, :x1)
# plotLocalProduct(fg, :x2)
#
# plotLocalProduct(fg, :l7, levels=3)
#
#
# plotLocalProduct(fg, :l1_0)
# plotLocalProduct(fg, :l2_0)
# plotLocalProduct(fg, :l3_0)
# plotLocalProduct(fg, :l4_0)
#
# plotLocalProduct(fg, :l4)
#
# ls(fg, :l7)
#
#
# drawPosesLandms(fg)


# ensureAllInitialized!(fg)
# cu, pr = predictVariableByFactor(fg, :l7, pp_x7l7_br, [:x7;:l7])
# if minkld(cu, pr) < 3
    # addFactor!(fg, [:x7, :l7], pp_x7l7_br, autoinit=autoinit)
# else
#     addVariable!(fg, :l7_1, Point2)
# end


##
