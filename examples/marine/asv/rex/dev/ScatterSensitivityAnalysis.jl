

using Statistics
using DataFrames
using JLD2, FileIO

using Cairo, Gadfly
Gadfly.set_default_plot_size(40cm,20cm)

## plotting

sap_ = getFactorType(newfg, :x0x1f1)

##

Caesar.plotScatterAlign(sap_, bw=0.001)

##

# snt_ = deepcopy(snt)
snt = overlayScatterMutate(sap_, sample_count=75, bw=5e-5) #, user_coords=snt_.best_coords)
plotScatterAlign(snt)




## sensitivity sweep on bw


SNT_08 = [[overlayScatterMutate(sap_; sample_count=150, bw) for i in 1:100] for bw in exp10.(-6:1:-1)];

mkdir("/tmp/caesar/rex/SNT_08")
@save "/tmp/caesar/rex/SNT_08/SNT_08.jld2" SNT_08

## histogram sensitivities

df = DataFrame(s=Int[], bw=Float64[], group=Symbol[], μ=Float64[], σ=Float64[], _μ=Float64[], μ_=Float64[] )

for (i,s) in enumerate(-6:-1)

  tl = "e$s"
  X_ = (x->x.best_coords[1]).(SNT[i])
  Y_ = (x->x.best_coords[2]).(SNT[i])
  R_ = (x->x.best_coords[3]).(SNT[i])

  μ_x = Statistics.mean(X_); σ_x = Statistics.std(X_)
  μ_y = Statistics.mean(Y_); σ_y = Statistics.std(Y_)
  μ_r = Statistics.mean(R_); σ_r = Statistics.std(R_)
  
  _μx, μx_ = μ_x - σ_x, μ_x + σ_x
  _μy, μy_ = μ_y - σ_y, μ_y + σ_y
  _μr, μr_ = μ_r - σ_r, μ_r + σ_r

  push!( df, ( s,exp10(s),:x, μ_x,σ_x, _μx,μx_ ) )
  push!( df, ( s,exp10(s),:y, μ_y,σ_y, _μy,μy_ ) )
  push!( df, ( s,exp10(s),:r, μ_r,σ_r, _μr,μr_ ) )
  
  # Hx = Gadfly.plot(x=X_, Geom.histogram, Guide.title("X, $tl"))
  # Hy = Gadfly.plot(x=Y_, Geom.histogram, Guide.title("Y, $tl"))
  # Ht = Gadfly.plot(x=R_, Geom.histogram, Guide.title("θ, $tl"))
  # hstack(Hx, Hy, Ht);
end

##

df_xy = filter( row -> row.group != :r, df )
df_r  = filter( row -> row.group == :r,  df )

p_xy = Gadfly.plot(df_xy, x=:bw, y=:μ, ymin=:_μ, ymax=:μ_, color=:group, Geom.line, Geom.errorbar, Scale.x_log10)
p_r  = Gadfly.plot(df_r,  x=:bw, y=:μ, ymin=:_μ, ymax=:μ_, color=:group, Geom.line, Geom.errorbar, Scale.x_log10)

hstack(p_xy, p_r)



##=============================================================================================
## dev testing on one scatter image with known transform

# sweeps_ = sweeps[5:6]
# sweeps_[2] = sweeps[5]

# domain = (range(-100,100,length=976),range(-100,100,length=976))

# newfg = buildGraphChain!( sweeps_,
#                           ScatterAlignPose2,
#                           (_, data) -> (data.currData,data.nextData,domain),
#                           stopAfter=1,
#                           doRef = false,
#                           inflation_fct=0.0,
#                           solverParams=SolverParams(graphinit=false, inflateCycles=1) )
# #

##

# fct = getFactorType(newfg, :x0x1f1)
# Xtup = sampleFactor(newfg, :x0x1f1)
# e0 = identity_element(SpecialEuclidean(2))
# δ = calcFactorResidualTemporary(fct, (Pose2,Pose2), Xtup, (e0, e0))


#