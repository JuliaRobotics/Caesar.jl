using RoME, Distributions, TransformUtils
# dev test script for Pose2 and Pose2Point2BearingRange using bss sampler
using RoMEPlotting

const TU = TransformUtils

# drive in a circle (copy hexagonal increase)


# start with an empty factor graph object
N = 100
fg = initfg()

# Add the first pose :x0
addVariable!(fg, :x0, Pose2)

# Add at a fixed location PriorPose2 to pin :x0 to a starting location
addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*eye(3))))

# Drive around in a hexagon
n_sides = 16
side_length = 3.0
for i in 0:(n_sides-1)
  psym = Symbol("x$i")
  nsym = Symbol("x$(i+1)")
  addVariable!(fg, nsym, Pose2)
  pp = Pose2Pose2(MvNormal([side_length;0;2pi/n_sides], diagm([0.01;0.01;0.001].^2)))
  addFactor!(fg, [psym;nsym], pp )
end

# two acoustic beacons

addVariable!(fg, :l1, Point2)
addVariable!(fg, :l2, Point2)

pL1 = IIF.Prior(MvNormal([50.0; 50.0], 0.1^2*eye(2)))
pL2 = IIF.Prior(MvNormal([-50.0; 50.0], 0.1^2*eye(2)))
addFactor!(fg,[:l1],pL1)
addFactor!(fg,[:l2],pL2)

# doautoinit!(fg, :l1, N=N)
# doautoinit!(fg, :l2, N=N)
ensureAllInitialized!(fg)



# build probabilities from parametric Gaussian but use bss sampler

# calculate the range bearings from each pose to

GTposes = KDE.getKDEMax.(getVertKDE.(fg, ls(fg)[1]))
GTlandm = KDE.getKDEMax.(getVertKDE.(fg, ls(fg)[2]))

brDict = Dict{Symbol, Dict{Symbol,Any}}()

for poseidx in 1:length(GTposes), lmidx in 1:length(GTlandm)
    gtpose = GTposes[poseidx]
    landm = GTlandm[lmidx]

    px, py = gtpose[1], gtpose[2]
    lx, ly = landm[1], landm[2]
    delta = [landm[1]-gtpose[1];landm[2]-gtpose[2]]
    d = norm(delta)
    wyaw = atan2(delta[2], delta[1])
    # must still incorporate pose yaw
    ryaw = TU.wrapRad(wyaw - gtpose[3])

    # generate range measurement samplable belief
    r_rng = collect(linspace(0, 2*d, 128)) # range vector from 0 to two times the distance
    n_rng = Normal(d, 0.1)
    w_rng = pdf(n_rng, r_rng) # weights
    # TODO: check for weight renormalization
    bss_rng = IIF.AliasingScalarSampler(r_rng, w_rng)

    r_bear = collect(linspace(-pi, pi, 512)) # range vector from 0 to t
    n_bear = Normal(ryaw, 0.1)
    w_bear = pdf(n_bear, r_bear)
    bss_bear = IIF.AliasingScalarSampler(r_bear, w_bear)

    posesym = ls(fg)[1][poseidx]
    lmsym = ls(fg)[2][lmidx]
    if !haskey(brDict, posesym)
      brDict[posesym] = Dict{Symbol, Any}()
    end
    brDict[posesym][lmsym] = Pose2Point2BearingRange(bss_bear, bss_rng)
end


for posesym in [:x1], lmsym in [:l1;:l2]
  addFactor!(fg, [posesym;lmsym], brDict[posesym][lmsym])
end

writeGraphPdf(fg,engine="dot")

# t = string(now())
# savejld(fg, file="presolve_$t.jld")
IIF.batchSolve!(fg)
# savejld(fg, file="postsolve_$t.jld")


using Gadfly
# show that it all works
# poses
pl = drawPoses(fg, spscale=1.0)

# landmarks
pl = drawPosesLandms(fg, spscale=2.0)
Gadfly.draw(Gadfly.PDF("/tmp/test3.pdf", 20cm, 10cm),pl)


stuff = plotLocalProduct(fg, :x1)


# then use this to validate the Sandshark pipeline

# pvt: make this an example or test somewhere?
# loosen
