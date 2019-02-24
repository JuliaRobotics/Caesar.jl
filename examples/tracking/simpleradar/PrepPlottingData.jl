
TT = BallTreeDensity[]
TTa = BallTreeDensity[]
Mmean = Float64[]
Mmax = Float64[]
Amean = Float64[]
Amax = Float64[]

K = 0:(nposes-1)
for k in K
  T0 = getKDE(fg, Symbol("t$k"))
  push!(TT, T0)
  T0a = marginal(T0, [2])
  push!(TTa, T0a)
  m0 = getKDEMean(T0)
  push!(Mmean, m0[1])
  mam0 = getKDEMax(T0)
  push!(Mmax, mam0[1])
  # angles
  push!(Amean, m0[2])
  push!(Amax, mam0[2])
end

len = length(K)
mt = [reshape(ranges[1:len],:,1) reshape(Mmean,:,1) reshape(Mmax,:,1)]
angs = [reshape(angles[1:len],:,1) reshape(Amean,:,1) reshape(Amax,:,1)]


px_max = Mmax.*cos.(Amax)
py_max = Mmax.*sin.(Amax)

px_mean = Mmean.*cos.(Amean)
py_mean = Mmean.*sin.(Amean)

# Prep dataframes

dferr = vcat(
DataFrame(
  x = ts,
  y = (ranges[1:len] - Mmean),
  Legend = "mean"
),
DataFrame(
  x = ts,
  y = (ranges[1:len] - Mmax),
  Legend = "max"
) )

dferrang = vcat(
DataFrame(
  x = ts,
  y = AMP.difftheta.(angles[1:len], Amean),
  Legend = "mean"
),
DataFrame(
  x = ts,
  y = AMP.difftheta.(angles[1:len], Amax),
  Legend = "max"
) )


df = vcat(
DataFrame(
  x = ts,
  y = ranges[1:len],
  Legend = "true"
),
DataFrame(
  x = ts,
  y =  Mmean,
  Legend = "mean"
),
DataFrame(
  x = ts,
  y = Mmax,
  Legend = "max"
) )


dfa = vcat(
DataFrame(
  x = ts,
  y = angles[1:len],
  Legend = "true"
),
DataFrame(
  x = ts,
  y =  Amean,
  Legend = "mean"
),
DataFrame(
  x = ts,
  y = Amax,
  Legend = "max"
) )


dfcm = vcat(
DataFrame(
  x=XYdense[1,:],
  y=XYdense[2,:],
  Legend="true"
),
DataFrame(
  x = px_mean[2:end],
  y = py_mean[2:end],
  Legend="mean"
),
DataFrame(
  x = px_max,
  y = py_max,
  Legend="max"
)
)
