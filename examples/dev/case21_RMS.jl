using JLD2
@load "data/groundref_b8acd9_wynglas.jld2"
ground_ref = deepcopy(d)
varIds = collect(keys(ground_ref))
sort!(varIds, lt=DFG.natural_lt)
refx = []
refy = []
refθ = []
for idx in varIds
  push!(refx, ground_ref[idx].val[1])
  push!(refy, ground_ref[idx].val[2])
  push!(refθ, ground_ref[idx].val[3])
end

cd(@__DIR__)
using DistributedFactorGraphs
using RoME
## load FG
folder = "results/skuif_X_l7_8_9_14/mm/"
folder = "results/skuif_X_l7_8_init/mm/"
filels =  readdir(pwd()*"/"*folder)
sort!(filels, lt=natural_lt)
filels = filels[2:2:end]
n = length(filels)
big_ppi_mat_x = zeros(68,n)#Matrix{Float64}(undef, 68, 20)
big_ppi_mat_y = zeros(68,n)#Matrix{Float64}(undef, 68, 20)
big_ppi_mat_θ = zeros(68,n)#Matrix{Float64}(undef, 68, 20)
for (i,filein) in enumerate(filels)  
  fg = initfg()
  @info "loading: $(filename*filein)"
  loadDFG!(fg, folder*filein)  ##
  varIds = ls(fg)  
  sort!(varIds, lt=DFG.natural_lt)
    for (k, idx) in enumerate(varIds)
    # segppe = getVariableSolverData(fg, idx, :parametric).val
    # segppe = getPPE(fg, idx) |> getMeanPPE
    segppe = getPPE(fg, idx) |> getMaxPPE
    big_ppi_mat_x[k,i] = segppe[1]
    big_ppi_mat_y[k,i] = segppe[2]
    big_ppi_mat_θ[k,i] = segppe[3]
  end
end##using Plotserr_mat_x = Matrix{Float64}(undef, 68, n)
err_mat_y = Matrix{Float64}(undef, 68, n)
err_mat_θ = Matrix{Float64}(undef, 68, n)
Xrefx =  refx[9:end]
Xrefy =  refy[9:end]
Xrefθ =  refθ[9:end]
p = scatter(refx[1:8],refy[1:8])
scatter!(p, big_ppi_mat_x[1:2:16,1], big_ppi_mat_y[1:2:16,1])
scatter!(p, big_ppi_mat_x[2:2:16,1], big_ppi_mat_y[2:2:16,1])
Xbig_ppi_mat_x =  big_ppi_mat_x[17:end,:]
Xbig_ppi_mat_y =  big_ppi_mat_y[17:end,:]
Xbig_ppi_mat_θ =  big_ppi_mat_θ[17:end,:]
err_mat_x = Matrix{Float64}(undef, 52, n)
err_mat_y = Matrix{Float64}(undef, 52, n)
err_mat_θ = Matrix{Float64}(undef, 52, n)
for i = 1:52
  err_mat_x[i,:] = Xrefx[i] .- Xbig_ppi_mat_x[i,:]
  err_mat_y[i,:] = Xrefy[i] .- Xbig_ppi_mat_y[i,:]
  err_mat_θ[i,:] = Xrefθ[i] .- Xbig_ppi_mat_θ[i,:]
end
for i = 1:68
  err_mat_x[i,:] = big_ppi_mat_x[i,1] .- big_ppi_mat_x[i,:]
  err_mat_y[i,:] = big_ppi_mat_y[i,1] .- big_ppi_mat_y[i,:]
  err_mat_θ[i,:] = big_ppi_mat_θ[i,1] .- big_ppi_mat_θ[i,:]
end
Plots.plot(err_mat_x[9:end,:]')
Plots.plot(err_mat_x[1:9,:]')
Plots.plot(big_ppi_mat_x)
Plots.plot(big_ppi_mat_y)
Plots.plot(big_ppi_mat_θ)
noises = collect(noiselevels)
noises = collect(multihypos)
# mx = Plots.plot(noises, sqrt.(mean(err_mat_x[9:end,:].^2; dims=1))', labels = "Mean Error x")
mx = Plots.plot(noises, sqrt.(mean(err_mat_x[9:end,:].^2; dims=1))', labels = "mm-x")
# mx = Plots.plot(noises, mean(err_mat_x[9:end,:]; dims=1)', labels = "")
# mc = Plots.plot!(mx, noises, sqrt.(mean(err_mat_y[9:end,:].^2; dims=1))', labels="Mean Error y")
mc = Plots.plot!(mx, noises, sqrt.(mean(err_mat_y[9:end,:].^2; dims=1))', labels="par-x")
xlabel!("Offset on landmarks l7 and l8 [m]")
ylabel!("RMS perturbation error over all poses [m]")
ylims!(0,0.15)
xlims!(0,0.5)
savefig("plots/skuif_X_l7_8_init/mm.pdf")
Plots.plot!(mc, )
Plots.plot(err_mat_θ)
ground_ref[:l10].val


