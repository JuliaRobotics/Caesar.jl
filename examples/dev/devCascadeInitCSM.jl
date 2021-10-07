# develop cascade tree init

# using Caesar, RoME, Images
using Caesar, RoME


fg = generateCanonicalFG_Hexagonal(graphinit=false)

getSolverParams(fg).treeinit = true
getSolverParams(fg).graphinit = false
getSolverParams(fg).limititers = 100

getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
# getSolverParams(fg).drawtreerate = 0.25
getSolverParams(fg).dbg = true
getSolverParams(fg).async = true


# limitcliqs = [:x0=>8;:x4=>12;:l1=>21;:x1=>21;:x5=>50;:x3=>50] # breaks
# limitcliqs = [:x0=>8;:x4=>13;:l1=>21;:x1=>21;:x5=>60;:x3=>60] # 50 # doesnt break, blocks

# injectDelayBefore=[2=>(canCliqMargRecycle_StateMachine=>5); ] # step 8
# injectDelayBefore=[5=>(canCliqMargRecycle_StateMachine=>5); ]
# injectDelayBefore=[6=>(towardUpOrDwnSolve_StateMachine=>10); ]
# injectDelayBefore = nothing



mkpath(getLogPath(fg))
verbosefid = open(joinLogPath(fg, "csmVerbose.log"),"w")
# verbosefid = stdout
tree = solveTree!(fg, recordcliqs=ls(fg), verbose=true, verbosefid=verbosefid, timeout=50 ) #, timeout=40 , injectDelayBefore=injectDelayBefore ) #, limititercliqs=limitcliqs);


flush(verbosefid)
close(verbosefid)
# for .async = true (because .drawTree=true)
smt[7] |> x->schedule(x, InterruptException(), error=true)


open(joinLogPath(fg, "csmLogicalReconstructMax.log"),"w") do io
  IIF.reconstructCSMHistoryLogical(getLogPath(fg), fid=io)
end

# async case
fetchCliqHistoryAll!(smt, hists)

open(joinLogPath(fg, "csmSequ.log"),"w") do fid
  printCliqHistorySequential(hists, nothing, fid)
end

open(joinLogPath(fg, "csmLogi.log"),"w") do fid
  printCSMHistoryLogical(hists, fid)
end



# printCliqHistorySequential(hists)
# printCliqHistorySequential(hists, 1=>10)
# printCliqHistorySequential(hists, [1,4,6]=>11:15)
# printCliqHistorySequential(hists, [1=>9:16; 2=>20:34; 4=>29:34])
# printCliqHistorySequential(hists, [5=>12:21;6=>12:21])


printCSMHistoryLogical(hists)


# also see dbg logs at this path for more info
# @show getLogPath(fg)


using Images
csmAnimateSideBySide(tree, hists, encode=true, nvenc=true, show=true)



# fps = 5
# run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/both_%d.png -c:v libtheora -vf fps=$fps -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -q 10 /tmp/caesar/csmCompound/out.ogv`)
# @async run(`totem /tmp/caesar/csmCompound/out.ogv`)

using Logging

fnc_ = hists[2][end][3]
hist_ = hists[2][end][4] |> deepcopy;

getCliqueData(hist_.cliq).solveCondition = Condition()
hist_.logger = SimpleLogger(stdout)




fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)
fnc_ = fnc_(hist_)




## dev


open(joinLogPath(fg, "csmLogicalReconstruct.log"),"w") do io
  IIF.reconstructCSMHistoryLogical(getLogPath(fg), fid=io)
end





## everything in csm2
# canCliqMargRecycle_StateMachine
# canCliqIncrRecycle_StateMachine
# isCliqUpSolved_StateMachine
# buildCliqSubgraph_StateMachine
# canCliqMargSkipUpSolve_StateMachine
# blockUntilChildrenHaveStatus_StateMachine
# trafficRedirectConsolidate459_StateMachine
# maybeNeedDwnMsg_StateMachine
# doesParentNeedDwn_StateMachine
# determineCliqNeedDownMsg_StateMachine
# slowIfChildrenNotUpSolved_StateMachine

## first steps in csm5
# canCliqMargRecycle_StateMachine
# canCliqIncrRecycle_StateMachine
# isCliqUpSolved_StateMachine
# buildCliqSubgraph_StateMachine
# canCliqMargSkipUpSolve_StateMachine
# blockUntilChildrenHaveStatus_StateMachine
# trafficRedirectConsolidate459_StateMachine
# maybeNeedDwnMsg_StateMachine
# doesParentNeedDwn_StateMachine
# determineCliqNeedDownMsg_StateMachine
# towardUpOrDwnSolve_StateMachine


#