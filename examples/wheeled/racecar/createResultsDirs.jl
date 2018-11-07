
mkdir(resultsdir)
mkdir(resultsdir*"/tags")
mkdir(resultsdir*"/images")


fid = open(resultsdir*"/readme.txt", "w")
println(fid, datafolder)
println(fid, camidxs)
close(fid)

fid = open(resultsparentdir*"/racecar.log", "a")
println(fid, "$(currdirtime), $datafolder, $(camidxs)")
close(fid)
