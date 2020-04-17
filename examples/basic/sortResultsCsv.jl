# sort results

using DelimitedFiles
using DistributedFactorGraphs

@show ARGS

data = readdlm(ARGS[1], ',')
Base.cp(ARGS[1], ARGS[1]*"_backup")

usPoses = Symbol.(data[:,1])

poses = usPoses |> sortDFG

sortperm = zeros(Int, length(poses))
for i in 1:length(poses)
  sortperm[i] = findfirst(x->x==poses[i], usPoses)
end

sortperm

sdata = data[sortperm, :]

writedlm(ARGS[1], sdata, ',')

#
