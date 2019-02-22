
rstd = 3.0
astd = 0.1

rrm = 5.0
rrstd = 15.0
aam = 0.0
aastd = 0.2

N = 100
fg = initfg()

addVariable!(fg, :t0, Polar)
addFactor!(fg, [:t0], PriorPolar(Normal(ranges[1],rstd), Normal(angles[1],astd)) )

for tidx in 1:(nposes-1)
  psym = Symbol("t$(tidx-1)")
  sym = Symbol("t$tidx")
  addVariable!(fg, sym, Polar)
  addFactor!(fg, [sym], PriorPolar(Normal(ranges[tidx+1],rstd), Normal(angles[tidx+1],astd)) )
  addFactor!(fg, [psym; sym], PolarPolar(Normal(rrm, rrstd), Normal(aam, aastd)) )
end
