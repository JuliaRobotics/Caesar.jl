# errors from LBL, remove post interpolation outliers

using Statistics

## Load necessary code

using Pkg
Pkg.activate(@__DIR__)
pkg"instantiate"
pkg"precompile"


using FileIO


include(joinpath(@__DIR__, "CommonUtils.jl"))
# include(joinpath(@__DIR__, "Plotting.jl"))


## load specific case


pargs = if !isdefined(Main, :parsed_args)
  parse_commandline()
else
  parsed_args
end

##

pargs["reportDir"] = if false && !haskey(pargs, "reportDir") && isdefined(Main, :fg)
  getLogPath(fg)
else
  # "/tmp/caesar/2020-02-23T01:43:32.222/fg_before_x291.tar.gz"
  # "/tmp/caesar/2020-02-24T04:22:23.282/fg_after_x1391.tar.gz"
  "/tmp/caesar/2020-02-24T04:19:39.943/fg_after_x1391.tar.gz"
end


# @show pargs["reportDir"]
# @show splitpath(pargs["reportDir"])

# load the factor graph needed
fg = if true
  println("going to load fg")
  fg = LightDFG{SolverParams}(params=SolverParams())
  @show pathElem = splitpath(pargs["reportDir"])
  @show getSolverParams(fg).logpath = joinpath(pathElem[1:end-1]...)
  loadDFG(pargs["reportDir"], Main, fg)
  fg
else
  fg
end


# random temp data
drt_data, TTm, XXm, YYm, XXf, YYf = loadResultsDRT(fg)



##

posesyms = ls(fg, r"x\d") |> sortDFG
filter!(x->isInitialized(fg, x), posesyms)
filter!(x->solverData(getVariable(fg, x), :lbl) != nothing, posesyms)
Xlbl = (x->(solverData(getVariable(fg, x), :lbl).val[1,1])).(posesyms)
Ylbl = (x->(solverData(getVariable(fg, x), :lbl).val[2,1])).(posesyms)

# pose PPE
Xppe = (x->(getVariablePPE(getVariable(fg, x)).suggested[1])).(posesyms)
Yppe = (x->(getVariablePPE(getVariable(fg, x)).suggested[2])).(posesyms)


# timestamp
ts = (x->getTimestamp(getVariable(fg, x))).(posesyms) .|> datetime2unix
T0 = ts[1]
ts .-= ts[1]
ts[end]


# filter on timestamps
# remove 420-450s
mask = 420 .< ts .< 450
imask = xor.(mask, true)

Xlbl[mask] .= Inf
Ylbl[mask] .= Inf

Gadfly.plot(x=ts, y=Xppe, Geom.line)
Gadfly.plot(x=ts, y=Yppe, Geom.line)

lblErr2 = (Xppe-Xlbl).^2 + (Yppe-Ylbl).^2
@show Statistics.mean(lblErr2[imask]) |> sqrt
@show cov(sqrt.(lblErr2[imask])) |> sqrt


##


Gadfly.plot(x=Xppe-Xlbl, Geom.histogram)
Gadfly.plot(x=ts, y=Xppe-Xlbl, Geom.line)
Gadfly.plot(x=ts, y=Yppe-Ylbl, Geom.line)

Gadfly.plot(x=ts, y=sqrt.(lblErr2), Geom.line)

findVariableNearTimestamp(fg,unix2datetime(T0+580))


## ranges nearby x580

ls(fg, :x581)


reportFactors(fg, Pose2Point2Range) #, [:x581l1f1; :x454l1f1])


ls(fg, :x454)



##  Plot LBL and belief only





drt_data, TTm, XXm, YYm, XXf, YYf = loadResultsDRT(fg)

mask = YYf .< -32
XXfm = XXf[mask]
YYfm = YYf[mask]
# only start does not match
TTmm = sum(mask)<length(TTm) ? TTm[end-sum(mask)+1:end] : TTm

# plot DRT
pl = Gadfly.plot(x=XXm[1:10:end], y=YYm[1:10:end], Geom.path) #, Theme(default_color=colorant"khaki"))

drtt = datetime2unix.(TTmm)
drtt .-= drtt[1]

pl = Gadfly.plot(
  Gadfly.layer(x=ts, y=Xppe, Geom.line, Theme(default_color=colorant"black")),
  Gadfly.layer(x=drtt, y=XXfm, Geom.line, Theme(default_color=colorant"red"))
)




#
