# include("../datasets/Vic120MM.jl")

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

function addLandmarksFactorGraph!(fg::G,
                                 f,
                                 idx,
                                 prevn,
                                 nextn;
                                 lcmode=:mmodal,
                                 lsrNoise=diagm([0.1;1.0]),
                                 N::Int=100,
                                 MM=nothing  ) where G <: AbstractDFG
    #
    @show I = intersect(keys(f[idx-1]),keys(f[idx]))
    for i in I
      pfez = f[idx-1][i]
      fez = f[idx][i]
      if abs(fez[2]) < (pi/2.0+5.0/180.0*pi)   # only add those recently seen by the laser, and not purely propagated way behind car
        lsy = string("l",i)
        if !haskey(MM,i)
          if !haskey(fg.IDs, lsy)    # has landmark
            # add from previous and latest factor here
            @show string(prevn[1]), lsy
            projNewLandm!(fg, string(prevn[1]), lsy, [pfez[2];pfez[1]], lsrNoise , N=N)
            addBRFG!(fg, string(nextn), lsy, [fez[2];fez[1]], lsrNoise)
          else
            # previous already added, only add the new factor here
            addBRFG!(fg, string(nextn), lsy, [fez[2];fez[1]], lsrNoise)
          end
        else
          lsymm = string("l",MM[i])
          if lcmode == :mmodal
            println("Adding bimodal factor")
            if !haskey(fg.IDs, lsy)
              # vlm = projNewLandm!(fg, string(prevn), lsy, [pfez[2];pfez[1]], lsrNoise, addfactor=false, tags=[:LANDMARK;])
              addMMBRFG!(fg, Symbol.([string(prevn[1]);lsymm;lsy]), [pfez[2];pfez[1]], lsrNoise, w=[0.5;0.5] )
            end
            addMMBRFG!(fg, Symbol.([string(prevn[1]);lsymm;lsy]), [fez[2];fez[1]], lsrNoise, w=[0.5;0.5] )
          elseif lcmode == :unimodal
            println("adding unimodal loop closure")
            addBRFG!(fg, string(nextn), lsymm, [fez[2];fez[1]], lsrNoise)
          end
        end
      end
    end

    # error("addLandmarksFactoGraph! -- not implemented yet")
    nothing
end

#@time d,f,DBG = advOdoByRules(DRS[1:2000,:],LsrFeats,distrule=15.0,yawrule=pi/6.0);
#progressExamplePlot(dOdo, LsrFeats, toT=100)
#plotParVicPrk(DRS)


function appendFactorGraph!(fg::G, d, f;
                  toT=Inf, fgpdf=false, lcmode=:mmodal,
                  lsrNoise=diagm([0.1;1.0]), P0=diagm([0.01;0.01;0.001]), Podo=diagm([0.5;0.5;0.005]),
                  idx::Int=1, N::Int=100, MM=Union{}) where G <: AbstractDFG

  len = length(d)
  T = d[idx][4]
  if idx==1
    # init pose
    prevn = initFactorGraph!(fg, init=d[idx][1:3])
  else
    v,X,nextn = getLastPose2D(fg)
    @show prevn = String(v.label)
  end

  while T < toT && idx <= len
    idx += 1
    T = idx <= len ? d[idx][4] : break
    if T > toT
      break
    end

    # add new poses via odom
    prev, X, nextn = getLastPose2D(fg)
    vp, fp = addOdoFG!(fg, nextn, d[idx][1:3], Podo, N=N)

    # add landmarks
    addLandmarksFactorGraph!(fg, f, idx, prevn, nextn, lcmode=lcmode, lsrNoise=lsrNoise, N=N, MM=MM)
    prevn = nextn
  end
  if fgpdf   writeGraphPdf(fg); end
  return idx
end


function doBatchRun(d, f; toT=30)
  fg = initfg()
  appendFactorGraph!(fg, d, f; toT=toT);
  p = drawPosesLandms(fg)
  tree = prepBatchTree!(fg);
  writeGraphPdf(fg); # make pdf of factor graph so we can see
  inferOverTree!(fg, tree)
  return fg, tree, p
end

# function saveImgSeq(d::Dict{Int,Dict{Int,Feature}}, lsrFeats::Dict{Int,LaserFeatures}; from::Int=1,to::Int=10,step::Int=1)
#   for i in from:step:to
#     p = drawFeatTrackers(d[i], lsrFeats[i].feats);
#     Gadfly.draw(PNG(string("imgs/img",i,".png"),35cm,25cm),p)
#   end
#   nothing
# end
#
#
# function drawIROSVicPrkFig(fgl::G, d, f, T, isamdict, fgu) where G <: AbstractDFG
#   p1 = progressExamplePlot(d,f,toT=T);
#   p2 = drawCompPosesLandm(fg,isamdict, fgu, lbls=false,drawunilm=false);
#
#   vstack(p1,p2)
# end
