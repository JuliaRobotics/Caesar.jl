# develop cascade tree init

using Caesar, RoME, Images


fg = generateCanonicalFG_Hexagonal(graphinit=false)

getSolverParams(fg).treeinit = true
getSolverParams(fg).graphinit = false
getSolverParams(fg).limititers = 100


getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
# getSolverParams(fg).drawtreerate = 0.25
# getSolverParams(fg).dbg = true


tree, smt, hists = solveTree!(fg, recordcliqs=ls(fg), verbose=true);


fid = open(joinLogPath(fg, "csm.log"),"w")
printCliqHistorySequential(hists, nothing, fid)
close(fid)


# printCliqHistorySequential(hists)
# printCliqHistorySequential(hists, 1=>10)
# printCliqHistorySequential(hists, [1,4,6]=>11:15)

printCliqHistorySequential(hists, [1=>10:15; 2=>45:52])



# also see dbg logs at this path for more info
# @show getLogPath(fg)


csmAnimateSideBySide(tree, hists, encode=true, nvenc=true, show=true)




# fps = 5
# run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/both_%d.png -c:v libtheora -vf fps=$fps -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -q 10 /tmp/caesar/csmCompound/out.ogv`)
# @async run(`totem /tmp/caesar/csmCompound/out.ogv`)



#