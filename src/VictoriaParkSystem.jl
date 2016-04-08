

function triggerPose(x, xprev, Tnow, Tprev,
                    distrule, timerule, yawrule)

  if norm(x[1:2]-xprev[1:2]) >= distrule
    @show Tnow, round(x,2), round(xprev,2)
    @show norm(x[1:2]-xprev[1:2])
    return 1
  end
  if abs(x[3]-xprev[3]) >= yawrule
    @show Tnow, round(x,2), round(xprev,2)
    @show abs(x[3]-xprev[3])
    return 2
  end
  if Tnow-Tprev > timerule
    return 3
  end
  return 0
end

type InSituSystem
  x::Array{Float64,1}
  dOdo::Dict{Int64,Array{Float64,1}}
  FeatAssc::Dict{Int64, Dict{Int64,Array{Float64,1}}}
  Tprev::Float64
  T0::Float64
  poseid::Int64
  wTbk1::Array{Float64,2}
  bk1Tbk::Array{Float64,2}
  lstlaseridx::Int64
  trackers::Dict{Int64,Feature}
end

function makeInSituSys(x::Array{Float64,1}, bfts0::Array{Float64,2})
  dOdo = Dict{Int64,Array{Float64,1}}()
  FeatAssc = Dict{Int64, Dict{Int64,Array{Float64,1}}}()
  Tprev = 0.0
  T0 = 0.0
  wTbk1 = SE2(x)
  bk1Tbk = SE2(zeros(3))
  poseid = 1
  dOdo[poseid] = [x[1];x[2];x[3];T0;0]
  lstlaseridx = 1
  trackers = initTrackersFrom(bfts0)
  return InSituSystem(
  x,
  dOdo,
  FeatAssc,
  Tprev,
  T0,
  poseid,
  wTbk1,
  bk1Tbk,
  lstlaseridx,
  trackers
  )
end

function poseTrigAndAdd!(instSys::InSituSystem, Ts::Float64,
                        distrule::Float64, timerule::Float64, yawrule::Float64)
  rule = triggerPose(instSys.x, zeros(3), Ts, instSys.Tprev, distrule, timerule, yawrule)
  if rule != 0
    instSys.bk1Tbk = SE2(instSys.x)
    n = [instSys.x;[Ts;rule]]
    instSys.poseid +=1
    instSys.dOdo[instSys.poseid] = n
    instSys.wTbk1 = instSys.wTbk1*instSys.bk1Tbk
    instSys.Tprev = Ts
    instSys.x[1] = 0.0; instSys.x[2]=0.0; instSys.x[3]=0.0
    return true
  end
  return false
end



function processTreeTrackersUpdates!(instSys::InSituSystem, lsrFeats::Dict{Int64,VictoriaParkTypes.LaserFeatures},
                                    Ts::Float64, b1Dxb::Array{Float64,1}, DBG=Union{}; dbgflag=true)
  propAllTrackers!(instSys.trackers, b1Dxb, [0.05;0.05;0.004])
  newlsridx, Ta = getFeatsAtT(lsrFeats, Ts, prev=instSys.lstlaseridx)
  if newlsridx != instSys.lstlaseridx
    # we have a new laser scan available and should update the trackers
    instSys.lstlaseridx = newlsridx
    bfts = lsrFeats[instSys.lstlaseridx].feats
    hardassc = assocMeasWFeats!(instSys.trackers, bfts)
    measUpdateTrackers!(instSys.trackers, hardassc, [0.5;0.05])
    if dbgflag
      DBG[instSys.lstlaseridx] = deepcopy(instSys.trackers)
    end
  end

  nothing
end

function advOdoByRules(DRS::Array{Float64,2}, lsrFeats::Dict{Int64,VictoriaParkTypes.LaserFeatures};
                        distrule=20.0, timerule=30.0, yawrule=pi/3.0, trkfeats=true)
  # DBG = Dict{Int,Dict{Int64,Array{BallTreeDensity,1}}}()
  # DBG = Dict{Int,Dict{Int64,Feature}}()
  DBG = Union{}

  bfts0 = lsrFeats[1].feats
  instSys = makeInSituSys(zeros(3), bfts0)
  fdict = Dict{Int64,Array{Float64,1}}()
  for f in instSys.trackers
    # if length(f[2].lastz) < 3
    #   error("HEY!! $(f[1])")
    # end
    fdict[f[2].id] = f[2].lastz
  end
  instSys.FeatAssc[instSys.poseid] = fdict
  for i in 1:size(DRS,1)
    # do odo
    dt = DRS[i,1]-instSys.T0
    whlspd, strang = compensateRawDRS(vec(DRS[i,:]))
    bTbm = SE2(instSys.x)
    instSys.x=uteOdomEasy(instSys.x, whlspd, strang, dt)
    bTbp = SE2(instSys.x)

    # propagate, and maybe measure update, feature trackers
    if trkfeats
      #@show keys(instSys.trackers)
      bmTbp = vec(se2vee(inv(bTbm)*bTbp))
      @time processTreeTrackersUpdates!(instSys, lsrFeats, DRS[i,1], bmTbp, DBG,dbgflag=false)
    end
    # subsample motion tracks for constructing the factor graph SLAM solution
    if poseTrigAndAdd!(instSys, DRS[i,1], distrule, timerule, yawrule)
      fdict = Dict{Int64,Array{Float64,1}}()
      for f in instSys.trackers
        mpt = vec(Base.mean(getPoints(f[2].bel),2))
        r,b = c2p(mpt)
        fdict[f[2].id] = [r;b;f[2].lastz[3]]
      end
      instSys.FeatAssc[instSys.poseid] = fdict
    end
    # advance previous time point
    instSys.T0 = DRS[i,1]
  end
  return instSys.dOdo, instSys.FeatAssc, DBG
end



function progressExamplePlot(dOdo, lsrFeats; toT=Inf)
    len = length(dOdo)
    pose = SE2(zeros(3))
    lastpose = zeros(3)
    idx = 1
    T = dOdo[idx][4]
    lstlaseridx = 1
    WFTSX = Array{Float64,1}()
    WFTSY = Array{Float64,1}()
    WLBLS = ASCIIString[]

    lastX = Array{Float64,1}()
    lastY = Array{Float64,1}()

    while T < toT && idx <= len

      lastX = Array{Float64,1}()
      lastY = Array{Float64,1}()
      pose = pose*SE2(dOdo[idx][1:3]) # todo -- replace with inferred latest pose
      #@show idx, T, vec(pose[1:2,3])
      lastpose = vec(se2vee(pose))

      # lstlaseridx, Ta = getFeatsAtT(lsrFeats, T, prev=lstlaseridx)
      # bfts = lsrFeats[lstlaseridx].feats
      fe = lsrFeats[idx]
      bfts = zeros(3,length(fe))
      lbls = ASCIIString[]
      k = collect(keys(fe))
      for i in 1:length(fe)
        bfts[1:length(fe[k[i]]),i] = fe[k[i]]
        push!(lbls, "l$(k[i])")
      end

      if size(bfts,2) > 0
        if bfts[1,1] != 0.0 && bfts[2,1] != 0.0 && bfts[3,1] != 0.0
          wfts = rotateFeatsToWorld(bfts, pose)
          for i in 1:size(wfts,2)
              push!(WFTSX, wfts[1,i])
              push!(WFTSY, wfts[2,i])
              push!(WLBLS, lbls[i])
              push!(lastX, wfts[1,i])
              push!(lastY, wfts[2,i])
          end
        end
      end
      idx += 1
      if idx <= len
        T = dOdo[idx][4]
      end
    end
    p = plotPoseDict(dOdo,to=idx-1)
    l = Gadfly.layer(x=WFTSX, y=WFTSY, label=WLBLS, Geom.label, Geom.point, Gadfly.Theme(default_color=colorant"red"))
    push!(p.layers, l[1])
    l2 = Gadfly.layer(x=WFTSX, y=WFTSY, Geom.point, Gadfly.Theme(default_color=colorant"red"))
    push!(p.layers, l2[1])
    for i in 1:length(lastX)
      push!(p.layers, Gadfly.layer(x=[lastpose[1];lastX[i]], y=[lastpose[2];lastY[i]], Geom.line, Gadfly.Theme(default_color=colorant"magenta"))[1])
    end
    p
end


function plotTrckStep(DBG, i, fid, m)
  @show keys(DBG[i])
  pf = DBG[i][fid]
  arr = Array{BallTreeDensity,1}()
  for j in 1:3
    push!(arr, marginal(pf[j],[m]))
  end
  plotKDE(arr, c=["red";"green";"black"])
end

function loadVicPrkDataset(filename::AbstractString="datasets/VicPrk.jld")
  DRS,GPS,LsrFeats,d,f = jldopen(filename, "r") do file
    read(file, "DRS")
    read(file, "GPS")
    read(file, "LsrFeats")
    read(file, "d")
    read(file, "f")
  end
  return DRS,GPS,LsrFeats,d,f
end
