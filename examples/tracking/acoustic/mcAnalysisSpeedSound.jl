# MC analysis on the speed of sound

using Statistics

MAX = Dict{Int, Vector{Float64}}()
MEAN = Dict{Int, Vector{Float64}}()

for mc in 1:10, ss in 420:20:420 #270:20:410

    !haskey(MAX, ss) ? (MAX[ss]=Float64[]) : nothing
    !haskey(MEAN, ss) ? (MEAN[ss]=Float64[]) : nothing

    intensityLeft = beamformBasic(leftwav, lefttemplate, soundSpeed=ss)
    intensityRight = beamformBasic(rightwav, righttemplate, soundSpeed=ss)

    pcLeft = intensityToCircularKDE(intensityLeft, SNRfloor=0.8, N=200)
    pcRight = intensityToCircularKDE(intensityRight, SNRfloor=0.8, N=200)


    fg = initfg()

    # add two beacons
    addVariable!(fg, :l0, Point2, labels=["BEACON"])
    addFactor!(fg, [:l0], Prior(MvNormal(l0, lcov)))

    addVariable!(fg, :l1, Point2, labels=["BEACON"])
    addFactor!(fg, [:l1], Prior(MvNormal(l1, lcov)))

    # add unknown pose location
    addVariable!(fg, :x0, Pose2, labels=["POSE"])
    addFactor!(fg, [:x0], PartialPrior(Normal(0.0, 0.01), (2,)) )

    # add two bearing only measurements
    addFactor!(fg, [:x0; :l0], Pose2Point2BearingRange(pcLeft, Rayleigh(1.0)))
    addFactor!(fg, [:x0; :l1], Pose2Point2BearingRange(pcRight, Rayleigh(1.0)))

    getSolverParams(fg).N = 100
    solveTree!(fg)

    pts = getVal(fg, :x0)
    pts0 = pts[:,pts[1,:] .< 1.0]
    X00 = kde!(pts0[1,:])

    push!(MAX[ss], getKDEMax(X00)[1])
    push!(MEAN[ss], getKDEMean(X00)[1])

end

@save "/home/dehann/.julia/dev/Caesar/examples/tracking/acoustic/mc_final.jld2" MAX MEAN


for mm in MAX
  @show mm[1], Statistics.mean(mm[2]), std(mm[2])
  @show mm[1], Statistics.mean(MEAN[mm[1]]), std(MEAN[mm[1]])
end



using DataFrames

DF = DataFrame[]

for mm in keys(MAX)
  push!(DF, DataFrame(
    speed = mm,
    error = MEAN[mm],
    Legend = "mean"
  ) )
  # push!(DF, DataFrame(
  #   speed = mm,
  #   error = MAX[mm],
  #   Legend = "Max"
  # ) )
end

alldata = vcat(DF...)

pl = Gadfly.plot(alldata, x=:speed, y=:error, Geom.boxplot)

pl |> SVG("/home/dehann/.julia/dev/Caesar/examples/tracking/acoustic/soundSpeed.svg",10cm,6cm)



## Linear regression

using GLM


alldata

ols = lm(@formula(error ~ speed), alldata)

round.(stderror(ols), digits=5)


gg = (x) -> 0.00231747*x - 0.815986

0.815986/0.00231747


using Colors

ll = Gadfly.layer(gg, 200, 450, Theme(default_color=colorant"red"))

push!(pl.layers, ll[1])

pl

#
