# errors from LBL, remove post interpolation outliers

using Statistics


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
Statistics.mean(lblErr2[imask]) |> sqrt
cov(sqrt.(lblErr2[imask])) |> sqrt

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
