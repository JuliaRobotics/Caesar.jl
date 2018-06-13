using Caesar, IncrementalInference, RoME, HDF5, JLD
using CloudGraphs, Neo4j


# # Uncomment out for command line operation
# cloudGraph, addrdict = standardcloudgraphsetup(nparticles=true)
# session = addrdict["session"]

# interactive operation
# session = "SESSROX"
# Nparticles = 100
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))


# get all the data

d = jldopen("test/data/VicPrkBasic.jld", "r") do file
  read(file, "dBasic")
end

f = jldopen("test/data/VicPrkBasic.jld", "r") do file
  read(file, "fBasic")
end
# MM = jldopen("test/data/VicPrkBasic.jld", "r") do file
#   read(file, "MMBasic")
# end
include("test/data/Vic120MM.jl")
MMr = MMwithRedirects(MM);

isamdict = jldopen("test/data/fgWithISAMrefERR01.jld", "r") do file
  read(file, "isamdict")
end
fgu = jldopen("test/data/fgWithISAMrefERR01.jld", "r") do file
  read(file, "fgu")
end


# do inference

T=100
fg = emptyFactorGraph();
fg.cg = cloudGraph

idx = appendFactorGraph!(fg, d, f, toT=T, lcmode=:mmodal, MM=MMr);



# tree = prepBatchTree!(fg, ordering=:qr,drawpdf=true);
# inferOverTree!(fg,tree, N=100);
#
# tree = wipeBuildNewTree!(fg)
# removeGenericMarginals!(conn)
# inferOverTree!(fg, tree)
