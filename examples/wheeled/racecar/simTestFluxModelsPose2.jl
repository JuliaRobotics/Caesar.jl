# basic parameter and prediction tests

using DelimitedFiles
using JLD2
using TensorCast
using Flux
using RoME

using Gadfly
Gadfly.set_default_plot_size(35cm,20cm)


include(joinpath((@__DIR__), "LoadPyNNTxt.jl"))

allModels = []
for i in 0:99
# /home/dehann/data/racecar/results/conductor/models/retrained_network_weights0
  # push!(allModels, loadPose2OdoNNModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/models/retrained_network_weights$i") )
  push!(allModels, loadPose2OdoNNModelIntoFlux(ENV["HOME"]*"/data/racecar/results/conductor/sim_models/sim_network_weights$i") )
end


allModels[1]


# load sim data

# thestring = joinpath(ENV["HOME"],"Documents","learning","sim5data.jld2") |> string
@load "/home/dehann/Documents/learning/sim5data.jld2"

sim5Inputs

# check consistency of all input data
for i in 2:4, j in 1:50
  @assert sim5Inputs[1][j] - sim5Inputs[i][j] |> norm < 1e-10 "input data inconsistent"
end


modelnum = 4
pred5 = (allModels[modelnum]).(sim5Inputs[modelnum])

@cast out5new[row,col] := pred5[1,row][col]
pln = Gadfly.plot(x=out5new[:,1],y=out5new[:,2],Geom.path)

@cast out5py[row,col] := sim5Outs[$modelnum][row][1,col]
plp = Gadfly.plot(x=out5py[:,1],y=out5py[:,2],Geom.path)

hstack(pln, plp)

# sim5Outs[1]




#
