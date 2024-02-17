


using Test
using IncrementalInference

## TESTED in Tutorial 1
# library used for graph visualizations
# using GraphPlot

## TESTED in RoMEPlotting
# using RoMEPlotting
# Gadfly.set_default_plot_size(20cm,15cm)

##
@testset "Test ICRA Tutorial 2 CJL API code" begin
##

# Start with an empty factor graph
fg = initfg()
getSolverParams(fg).useMsgLikelihoods = true

# add the first node
addVariable!(fg, :x0, ContinuousScalar)

# this is unary (prior) factor and does not immediately trigger autoinit of :x0.
addFactor!(fg, [:x0], Prior(Normal(0,1)))

# DFG.plotDFG(fg);

addVariable!(fg, :x1, ContinuousScalar)
# P(Z | :x1 - :x0 ) where Z ~ Normal(10,1)
addFactor!(fg, [:x0, :x1], LinearRelative(Normal(10.0,1)))

# allow the first run some time to complete Julia JIT compiling of plottinng functions.
# plotBelief(fg, :x0)

solveGraph!(fg);

@test isapprox( getPPESuggested(fg, :x0)[1], 0; atol=1)
@test isapprox( getPPESuggested(fg, :x1)[1], 10; atol=1)

# plotBelief(fg, [:x0, :x1])

addVariable!(fg, :x2, ContinuousScalar)

mmo = Mixture(LinearRelative, 
              (slip=Rayleigh(5), noslip=Uniform(35,60)), 
              [0.4; 0.6])
addFactor!(fg, [:x1, :x2], mmo)

solveGraph!(fg);

# plotBelief(fg, [:x0, :x1, :x2, :x3])

@test isapprox( getPPESuggested(fg, :x0)[1], 0; atol=1)
@test isapprox( getPPESuggested(fg, :x1)[1], 10; atol=1)


X2 = getBelief(fg, :x2)
@test isapprox( sum((s->X2([s;])).([-50:0.1:0;])*0.1)[1], 0 ; atol=0.1 )
@test isapprox( sum((s->X2([s;])).([0:0.1:40;])*0.1)[1], 0.4; atol=0.2 )  # 40% case
@test isapprox( sum((s->X2([s;])).([40:0.1:80;])*0.1)[1], 0.6; atol=0.2 ) # 60% case

# plotBelief(fg, [:x0, :x1, :x2])

addVariable!(fg, :x3, ContinuousScalar)
addFactor!(fg, [:x2, :x3], LinearRelative(Normal(-50, 1)))

solveGraph!(fg);


@test isapprox( getPPESuggested(fg, :x0)[1], 0; atol=1)
@test isapprox( getPPESuggested(fg, :x1)[1], 10; atol=1)


X2 = getBelief(fg, :x2)
@test isapprox( sum((s->X2([s;])).([-50:0.1:0;])*0.1)[1], 0 ; atol=0.1 )
@test isapprox( sum((s->X2([s;])).([0:0.1:40;])*0.1)[1], 0.4; atol=0.2 )  # 40% case
@test isapprox( sum((s->X2([s;])).([40:0.1:80;])*0.1)[1], 0.6; atol=0.2 ) # 60% case

X3 = getBelief(fg, :x3)
@test isapprox( sum((s->X3([s;])).([-100:0.1:-50;])*0.1)[1], 0 ; atol=0.1 )
@test isapprox( sum((s->X3([s;])).([-50:0.1:-20;] )*0.1)[1], 0.4; atol=0.2 )  # 40% case
@test isapprox( sum((s->X3([s;])).([-10:0.1:30;]  )*0.1)[1], 0.6; atol=0.2 )  # 60% case


addFactor!(fg, [:x3, :x0], LinearRelative(Normal(30, 1)))

tree = solveGraph!(fg);

# plotBelief(fg, [:x0, :x1, :x2, :x3])

ppes = DFG.getPPESuggested.(fg, [:x0;:x1;:x2;:x3])

#

@test isapprox( getPPESuggested(fg, :x0)[1], 0; atol=1)
@test isapprox( getPPESuggested(fg, :x1)[1], 10; atol=1)
@test isapprox( getPPESuggested(fg, :x2)[1], 20; atol=2)
@test isapprox( getPPESuggested(fg, :x3)[1], -30; atol=2)




##
end



##