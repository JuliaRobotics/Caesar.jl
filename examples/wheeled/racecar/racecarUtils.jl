


# add AprilTag sightings from this pose
function addApriltags!(fg, pssym, posetags; bnoise=0.1, rnoise=0.1, lmtype=Point2, fcttype=Pose2Pose2, DAerrors=0.0, autoinit=true )
  @show currtags = ls(fg, r"l")
  for lmid in keys(posetags)
    @show lmsym = Symbol("l$lmid")
    if !(lmsym in currtags)
      @info "adding node $lmsym"
      addVariable!(fg, lmsym, lmtype)
    end
    ppbr = nothing
    if lmtype == RoME.Point2
      ppbr = Pose2Point2BearingRange(Normal(posetags[lmid][:bearing][1],bnoise),
                                     Normal(posetags[lmid][:range][1],rnoise))
    elseif lmtype == Pose2
      dx, dy = posetags[lmid][:bP2t].translation[1], posetags[lmid][:bP2t].translation[2]
      dth = convert(RotXYZ, posetags[lmid][:bP2t].linear).theta3
      # ppbr = Pose2Pose2(
      #           MvNormal([ posetags[lmid][:pos][3];
      #                     -posetags[lmid][:pos][1];
      #                      posetags[lmid][:tRYc]],
      #                    diagm([0.1;0.1;0.05].^2)) )
      ppbr = fcttype(
                MvNormal([dx;
                          dy;
                          dth],
                         Matrix(Diagonal([0.05;0.05;0.03].^2))) )
    end
    if rand() > DAerrors
      # regular single hypothesis
      addFactor!(fg, [pssym; lmsym], ppbr, autoinit=autoinit)
    else
      # artificial errors to data association occur
      info("Forcing bad data association with $lmsym")
      xx,ll = ls(fg)
      @show ll2 = setdiff(ll, [lmsym])
      @show daidx = round(Int, (length(ll2)-1)*rand()+1)
      @show rda = ll2[daidx]
      addFactor!(fg, [pssym; lmsym; rda], ppbr, multihypo=[1.0;0.5;0.5], autoinit=autoinit)
    end
  end
  nothing
end

function addnextpose!(fg,
                      prev_psid,
                      new_psid,
                      pose_tag_bag;
                      lmtype=Point2,
                      odotype=Pose2Pose2,
                      fcttype=Pose2Pose2,
                      DAerrors=0.0,
                      autoinit=true,
                      odopredfnc=nothing,
                      joysticktimeseries=nothing,
                      parametricOdoMix=0.3)
  #
  prev_pssym = Symbol("x$(prev_psid)")
  new_pssym = Symbol("x$(new_psid)")

  let odoKDE, DXmvn = MvNormal(zeros(3),diagm([0.4;0.1;0.4].^2))
    # predict delta x y th odo if able
    if odopredfnc != nothing && joysticktimeseries != nothing
      nnpts = odopredfnc(joysticktimeseries)
      # replace theta points
      nnpts[:,3] .= rand(DXmvn, size(nnpts,1))[:,3]
      mvnpts = rand( DXmvn, round(Int, parametricOdoMix*size(nnpts, 1)) )
      odoKDE = manikde!([nnpts;mvnpts], Pose2)
    else
      odoKDE = DXmvn
    end

    # first pose with zero prior
    if odotype == Pose2Pose2
      addVariable!(fg, new_pssym, Pose2)
      addFactor!(fg, [prev_pssym; new_pssym], Pose2Pose2(odoKDE), autoinit=autoinit)
    elseif odotype == VelPose2VelPose2
      addVariable!(fg, new_pssym, DynPose2(ut=round(Int, 200_000*(new_psid))))
      addFactor!(fg, [prev_pssym; new_pssym],
                  VelPose2VelPose2(odoKDE,
                                   MvNormal(zeros(2),Matrix(Diagonal([0.2;0.1].^2)))),
                 autoinit=autoinit)
      #
    end
  end

  addApriltags!(fg, new_pssym, pose_tag_bag, lmtype=lmtype, fcttype=fcttype, DAerrors=DAerrors)
  new_pssym
end


function prepCamLookup(imgseq; filenamelead="camera_image")
  # camcount = readdlm(datafolder*"$(imgfolder)/cam-count.csv",',')
  camlookup = Dict{Int, String}()
  count = -1
  for i in imgseq
    count += 1
    camlookup[count] = "$(filenamelead)$(i).jpeg"
    # camlookup[camcount[i,1]] = strip(camcount[i,2])
  end
  return camlookup
end



function detectTagsViaCamLookup(camlookup, imgfolder, imgsavedir)
  # AprilTag detector
  detector = AprilTagDetector()

  # extract tags from images
  IMGS = []
  TAGS = []
  psid = 1
  for psid in 0:(length(camlookup)-1)
    img = load("$(imgfolder)/$(camlookup[psid])")
    tags = detector(img)
    push!(TAGS, deepcopy(tags))
    push!(IMGS, deepcopy(img))
    foreach(tag->drawTagBox!(IMGS[psid+1],tag, width = 5, drawReticle = false), tags)
    Images.save(imgsavedir*"/tags/img_$(psid).jpg", IMGS[psid+1])
  end
  # psid = 1
  # img = load(datafolder*"$(imgfolder)/$(camlookup[psid])")

  # free the detector memory
  freeDetector!(detector)

  return IMGS, TAGS
end




function results2csv(fg; dir=".", filename="results.csv")

if !isdir(dir*"/results")
  mkdir(dir*"/results")
end
fid = open(dir*"/results/"*filename,"w")
for sym in [ls(fg)[1]...;ls(fg)[2]...]
  p = getVertKDE(fg, sym)
  val = string(KDE.getKDEMax(p))
  println(fid, "$sym, $(val[2:(end-1)])")
end
close(fid)

end




## BUILD FACTOR GRAPH FOR SLAM SOLUTION,


function main(WP,
              resultsdir::String,
              camidxs,
              tag_bagl;
              maxlen = (length(tag_bagl)-1),
              BB=10,
              N=100,
              lagLength=75,
              dofixedlag=true,
              jldfile::String="",
              failsafe::Bool=false,
              show::Bool=false,
              odopredfnc=nothing  )
#

# Factor graph construction
fg = initfg()
prev_psid = 0

# load from previous file
if jldfile != ""
  # fg, = loadjld( file=joinpath(resultsdir,jldfile) )
  fg = initfg()
  loadDFG(joinpath(resultsdir,jldfile), Main, fg)
  xx = ls(fg, r"x")
  prev_psid = parse(Int, string(xx[end])[2:end])
end

# fg.solverParams.isfixedlag = dofixedlag
# fg.solverParams.qfl = lagLength
defaultFixedLagOnTree!(fg, lagLength, limitfixeddown=false)

psid = 0
pssym = Symbol("x$psid")
# first pose with zero prior
addVariable!(fg, pssym, Pose2)
# addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))
addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),Matrix(Diagonal([0.01;0.01;0.001].^2)))) )
#
# addVariable!(fg, pssym, DynPose2(ut=0))
# # addFactor!(fg, [pssym], PriorPose2(MvNormal(zeros(3),diagm([0.01;0.01;0.001].^2))))
# addFactor!(fg, [pssym], DynPose2VelocityPrior(MvNormal(zeros(3),Matrix(Diagonal([0.01;0.01;0.001].^2))),
#                                               MvNormal(zeros(2),Matrix(Diagonal([0.1;0.05].^2)))))
#

addApriltags!(fg, pssym, tag_bagl[psid], lmtype=Pose2, fcttype=Pose2Pose2) # DynPose2Pose2

show ? drawGraph(fg, show=true) : nothing

# quick solve as sanity check
failsafe ? @warn("recursive failsafe no longer as option") : nothing
# , N=N, drawpdf=true, show=show, recursive=failsafe
tree, smt, hist = solveTree!(fg)

# add other positions


# Gadfly.push_theme(:default)

@sync begin

for psid in (prev_psid+1):1:maxlen
  # global prev_psid, maxlen
  @show psym = Symbol("x$psid")
  addnextpose!(fg, prev_psid, psid, tag_bagl[psid], lmtype=Pose2, odotype=Pose2Pose2, fcttype=Pose2Pose2, autoinit=true, odopredfnc=odopredfnc)
  # odotype=VelPose2VelPose2, fcttype=DynPose2Pose2
  # writeGraphPdf(fg)

  if psid % BB == 0 || psid == maxlen
    saveDFG(fg, resultsdir*"/racecar_fg_$(psym)_presolve")
    # , drawpdf=true, show=show, N=N, recursive=true
    tree, smt, hist = solveTree!(fg, tree, maxparallel=500)
  end

  T1 = remotecall(saveDFG, WP, fg, resultsdir*"/racecar_fg_$(psym)")
  @async fetch(T1)
  # saveDFG(fg, resultsdir*"/racecar_fg_$(psym)")

  ## save factor graph for later testing and evaluation
  ensureAllInitialized!(fg)
  T2 = remotecall(plotRacecarInterm, WP, fg, resultsdir, psym)
  @async fetch(T2)
  # plotRacecarInterm(fg, resultsdir, psym)

  # prepare for next iteration
  prev_psid = psid
end # for

# extract results for later use as training data
results2csv(fg, dir=resultsdir, filename="results.csv")

end #sync

return fg
end # main


#
