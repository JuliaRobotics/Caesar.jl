using HDF5, JLD


examplefolder = dirname(@__FILE__)
datafolder = joinpath(examplefolder,"..","..","data","datasets","victoriapark")
include(joinpath(examplefolder,"VicPrkEstimator.jl"))

d = jldopen(joinpath(datafolder,"VicPrkBasic.jld"), "r") do file
  read(file, "dBasic")
end
f = jldopen(joinpath(datafolder,"VicPrkBasic.jld"), "r") do file
  read(file, "fBasic")
end
# MM = jldopen("test/data/VicPrkBasic.jld", "r") do file
#   read(file, "MMBasic")
# end
include(joinpath(datafolder,"Vic120MM.jl"))
MMr = MMwithRedirects(MM);



# isamdict = jldopen(joinpath(datafolder,"fgWithISAMrefERR01.jld"), "r") do file
#   read(file, "isamdict")
# end
# fgu = jldopen(joinpath(datafolder,"fgWithISAMrefERR01.jld"), "r") do file
#   read(file, "fgu")
# end
