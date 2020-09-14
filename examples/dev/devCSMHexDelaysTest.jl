# develop cascade tree init

# using Caesar, RoME, Images

function csmHexDelayTest(injectDelayBefore)

  fg = generateCanonicalFG_Hexagonal(graphinit=false)

  getSolverParams(fg).treeinit = true
  getSolverParams(fg).graphinit = false
  getSolverParams(fg).limititers = 100
  getSolverParams(fg).async = true


  # limitcliqs = [:x0=>8;:x4=>12;:l1=>21;:x1=>21;:x5=>50;:x3=>50] # breaks
  # limitcliqs = [:x0=>8;:x4=>13;:l1=>21;:x1=>21;:x5=>60;:x3=>60] # 50 # doesnt break, blocks

  # injectDelayBefore=[5=>(testCliqCanRecycled_StateMachine=>5); ]
  # injectDelayBefore = nothing

  mkpath(getLogPath(fg))

  fid = open(joinLogPath(fg,"delays.dat"), "w")
  println(fid, injectDelayBefore)
  close(fid)

  verbosefid = open(joinLogPath(fg, "csmVerbose.log"),"w")
  # verbosefid = stdout
  tree, smt, hists = solveTree!(fg, recordcliqs=ls(fg), timeout=40, verbose=true, verbosefid=verbosefid, injectDelayBefore=injectDelayBefore); #, limititercliqs=limitcliqs);


  ## wait for async to complete

  println("Going to sleep an long period so that solve can finish or fail")
  sleep(180)

  flush(verbosefid)
  close(verbosefid)

  didFail = 0 < ((smt .|> x->x.state == :failed) |> sum)

  fid = open(joinLogPath(fg,"didFail.log"), "w")
  println(fid, "did the solve fail, $didFail")
  close(fid)

  # async case
  fetchCliqHistoryAll!(smt, hists)

  fid = open(joinLogPath(fg, "csmSequ.log"),"w")
  printCliqHistorySequential(hists, nothing, fid)
  close(fid)

  fid = open(joinLogPath(fg, "csmLogi.log"),"w")
  printCSMHistoryLogical(hists, fid)
  close(fid)

  return fg, didFail
end



#