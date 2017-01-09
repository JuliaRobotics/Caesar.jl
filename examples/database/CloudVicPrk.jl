using Caesar, IncrementalInference, RoME, HDF5, JLD
using CloudGraphs, Neo4j


# dbaddress = "mrg-liljon.csail.mit.edu"
dbaddress = "localhost"
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, "", "", "localhost", 27017, false, "", "");
cloudGraph = connect(configuration);

# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
IncrementalInference.setCloudDataLayerAPI!()
conn = cloudGraph.neo4j.connection



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
