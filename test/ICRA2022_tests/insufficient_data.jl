

using RoME
using Test
using JSON, JSON2


##

# Our dictionary of vehicle positions
GTp = Dict{Symbol, Vector{Float64}}()
GTp[:x0] = [0.0;0]
GTp[:x1] = [50.0;0]
GTp[:x2] = [100.0;0]
GTp[:x3] = [100.0;50.0]
GTp[:x4] = [100.0;100.0]
GTp[:x5] = [50.0;100.0]
GTp[:x6] = [0.0;100.0]
GTp[:x7] = [0.0;50.0]
GTp[:x8] = [0.0;-50.0]

# Our dictionary of landmark positions
GTl = Dict{Symbol, Vector{Float64}}()
GTl[:l1] = [10.0;30]
GTl[:l2] = [30.0;-30]
GTl[:l3] = [80.0;40]
GTl[:l4] = [120.0;-50];

refFdr = joinpath(@__DIR__, "test_data", "tut3")

function storeDistribution(file::AbstractString, bel::MKD)
  bel_ = packDistribution(bel)
  bel_s = JSON.json(bel_)
  fid = open(file, "w")
  println(fid, bel_s)
  close(fid)
end

function loadDistribution(file::AbstractString)
  fid = open(file, "r")
  # bel_s = read(fid)
  bel_s = JSON2.read(fid, PackedManifoldKernelDensity)
  close(fid)
  unpackDistribution(bel_s)
end


function checkTut3Posteriors(dfg::AbstractDFG, refFdr; atol=0.1, suffix="", vars=ls(dfg))
  for v in vars
    # reference belief
    ref = loadDistribution(joinpath(refFdr, string(v)*suffix*".json"))
    # test belief
    tb = getBelief(dfg, v)
    dist = mmd(ref, tb)
    @info "Test Tut3 "*string(v)*" "*suffix dist
    @test dist < atol
  end
end

function storeAllBeliefs(dfg, refFdr; suffix="", vars=ls(dfg))
  for v in vars
    B = getBelief(dfg, v)
    storeDistribution(joinpath(refFdr, string(v)*suffix*".json"), B)
  end
end

# This is a helper function that simulates how the robot moves and measures between ground truth positions. 
function vehicle_drives!(fgl::G, from_lbl::Symbol, to_lbl::Symbol, GTp::Dict, GTl::Dict; measurelimit::R=150.0) where {G <: AbstractDFG, R <: Real}
  currvar = listVariables(fgl)
  if !(to_lbl in currvar)
    println("Adding new variable $to_lbl")
    addVariable!(fgl, to_lbl, Point2)
    # an odometry distance factor
    @show rho = norm(GTp[from_lbl] - GTp[to_lbl])
    ppr = Point2Point2Range( Normal(rho, 3.0) )
    addFactor!(fgl, [from_lbl;to_lbl], ppr)
  else
    @warn "Variable node $to_lbl already in the factor graph."
  end
  beacons = keys(GTl)
  for ll in beacons
    rho = norm(GTl[ll] - GTp[to_lbl])
    # Add measurements to beacons/landmarks if within limit
    if rho < measurelimit
      ppr = Point2Point2Range( Normal(rho, 3.0) )
      if !(ll in currvar)
        println("Adding variable vertex $ll, not yet in fgl<:AbstractDFG.")
        addVariable!(fgl, ll, Point2)
      end
      addFactor!(fgl, [to_lbl;ll], ppr)
    end
  end
  nothing
end



##
@testset "Test ICRA2022 Tutorial 3" begin
##

# create the factor graph object
fg = initfg()
# getSolverParams(fg).N = 150

# first pose with no prior info about the initial numerical estimate
addVariable!(fg, :x0, Point2)

# add three landmarks
addVariable!(fg, :l1, Point2)
addVariable!(fg, :l2, Point2)
addVariable!(fg, :l3, Point2)

# and put priors on l1 and l2
addFactor!(fg, [:l1;], PriorPoint2(MvNormal(GTl[:l1], diagm(ones(2)))) )
addFactor!(fg, [:l2;], PriorPoint2(MvNormal(GTl[:l2], diagm(ones(2)))) )

# first range measurement from x0 to l1
rhoZ1 = norm(GTl[:l1]-GTp[:x0])
ppr = Point2Point2Range( Normal(rhoZ1, 2) )
addFactor!(fg, [:x0;:l1], ppr)

# second range measurement from x0 to l2
rhoZ2 = norm(GTl[:l2]-GTp[:x0])
ppr = Point2Point2Range( Normal(rhoZ2, 3.0) )
addFactor!(fg, [:x0; :l2], ppr)

# third range measurement from x0 to l3
rhoZ3 = norm(GTl[:l3]-GTp[:x0])
ppr = Point2Point2Range( Normal(rhoZ3, 3.0) )
addFactor!(fg, [:x0; :l3], ppr)


## ===========================================================================================
## PART _1

# the first run will be slow for JIT compilation to binary code (next runs are much faster)
solveGraph!(fg);
# plotBelief(fg, :x0, levels=5)
# storeAllBeliefs(fg,refFdr; suffix="_1")

##

checkTut3Posteriors(fg, refFdr; suffix="_1", vars=[:l1;:l2], atol=0.01)
checkTut3Posteriors(fg, refFdr; suffix="_1", atol=0.1)

## ===========================================================================================
## PART _2

#drive to location :x1, then :x2
vehicle_drives!(fg, :x0, :x1, GTp, GTl)
vehicle_drives!(fg, :x1, :x2, GTp, GTl)

##

solveGraph!(fg);
# storeAllBeliefs(fg,refFdr; suffix="_2") #, vars=ls(fg, r"l"))

##

# plotBelief(fg, [:x0;:x1;:x2], c=["red";"green";"blue"])
# plotBelief(fg, [:l3;:l4], c=["pink";"orange"])
checkTut3Posteriors(fg, refFdr; suffix="_2", vars=[:l1;:l2], atol=0.01)
checkTut3Posteriors(fg, refFdr; suffix="_2", vars=[:l3;:l4], atol=0.3)

checkTut3Posteriors(fg, refFdr; suffix="_2", vars=[:x0], atol=0.2)
checkTut3Posteriors(fg, refFdr; suffix="_2", vars=[:x1], atol=0.4)
checkTut3Posteriors(fg, refFdr; suffix="_2", vars=[:x2], atol=0.4)

## ===========================================================================================
## PART _3


vehicle_drives!(fg, :x2, :x3, GTp, GTl)
vehicle_drives!(fg, :x3, :x4, GTp, GTl)

##

solveGraph!(fg);
# storeAllBeliefs(fg,refFdr; suffix="_3") #, vars=ls(fg, r"l"))


##

# plotBelief(fg, sortDFG(ls(fg, r"x")))
# plotBelief(fg, [:l3;:l4], c=["pink";"orange"])
checkTut3Posteriors(fg, refFdr; suffix="_3", vars=[:l1;:l2], atol=0.01)
checkTut3Posteriors(fg, refFdr; suffix="_3", vars=[:l3;:l4], atol=0.5)

checkTut3Posteriors(fg, refFdr; suffix="_3", vars=[:x0], atol=0.2)
checkTut3Posteriors(fg, refFdr; suffix="_3", vars=[:x1], atol=0.3)
checkTut3Posteriors(fg, refFdr; suffix="_3", vars=[:x2], atol=0.3)
checkTut3Posteriors(fg, refFdr; suffix="_3", vars=[:x3;:x4], atol=0.5)


## ===========================================================================================
## PART _4

vehicle_drives!(fg, :x4, :x5, GTp, GTl)
vehicle_drives!(fg, :x5, :x6, GTp, GTl)


##

solveGraph!(fg);
# storeAllBeliefs(fg,refFdr; suffix="_4")

##

# plotBelief(fg, sortDFG(ls(fg, r"x")))
# plotBelief(fg, [:l3;:l4], c=["pink";"orange"])
checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:l1;:l2], atol=0.01)
checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:l3;:l4], atol=0.6)

checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:x0], atol=0.3)
checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:x1], atol=0.5)
checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:x2], atol=0.5)
checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:x3;:x4], atol=0.5)
checkTut3Posteriors(fg, refFdr; suffix="_4", vars=[:x5;:x6], atol=0.6)



## ===========================================================================================
## PART _5


vehicle_drives!(fg, :x6, :x7, GTp, GTl)
vehicle_drives!(fg, :x7, :x8, GTp, GTl)


##

solveGraph!(fg);
# storeAllBeliefs(fg,refFdr; suffix="_5")

##

# plotBelief(fg, sortDFG(ls(fg, r"x")))
# plotBelief(fg, [:l3;:l4], c=["pink";"orange"])

checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:l1;:l2], atol=0.01)
checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:l3;:l4], atol=0.8)

checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:x0], atol=0.7)
checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:x1], atol=0.7)
checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:x2], atol=0.7)
checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:x3;:x4], atol=0.7)
checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:x5;:x6], atol=0.8)
checkTut3Posteriors(fg, refFdr; suffix="_5", vars=[:x7;:x8], atol=0.7)



##
end



##