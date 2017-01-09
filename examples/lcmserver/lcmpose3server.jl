
using Caesar, RoME
using PyLCM

@pyimport hauv


function lcmaddodo!(slam::SLAMWrapper, odomsg)
  tA = odomsg[:trans]
  qA = odomsg[:rotation]
  qq = Quaternion(qA[1],qA[2:4])

  v,f = addOdoFG!(slam, Pose3Pose3( SE3(tA, qq), odomsg[:Cov]), ready=1)
  println("Added node $(v.label)")
  nothing
end




function runlistener(slam::SLAMWrapper)

  gg = (channel, msg_data) -> lcmaddodo!(fgl, msg_data)

  print("Preparing LCM...")
  lc = LCM()
  subscribe(lc, "NEWODOCONSTRAINT", gg)
  while true
    handle(lc)
  end
  println("done.")
  nothing
end

function setupSLAMinDB()
  slam = SLAMWrapper(initfg(sessionname="HAUV"), )
end
