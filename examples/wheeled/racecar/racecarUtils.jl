



function loadConfig()
  cfg = Dict{Symbol,Any}()
  data =   YAML.load(open(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cam_cal.yml")))
  bRc = eval(parse("["*data["extrinsics"]["bRc"][1]*"]"))
  # convert to faster symbol lookup
  cfg[:extrinsics] = Dict{Symbol,Any}()
  cfg[:extrinsics][:bRc] = bRc
  cfg[:intrinsics] = Dict{Symbol,Any}()
  cfg[:intrinsics][:height] = data["left"]["intrinsics"]["height"]
  cfg[:intrinsics][:width] = data["left"]["intrinsics"]["width"]
  haskey(data["left"]["intrinsics"], "camera_matrix") ? (cfg[:intrinsics][:cam_matrix] = data["left"]["intrinsics"]["camera_matrix"]) : nothing
  cfg[:intrinsics][:cx] = data["left"]["intrinsics"]["cx"]
  cfg[:intrinsics][:cy] = data["left"]["intrinsics"]["cy"]
  cfg[:intrinsics][:fx] = data["left"]["intrinsics"]["fx"]
  cfg[:intrinsics][:fy] = data["left"]["intrinsics"]["fy"]
  cfg[:intrinsics][:k1] = data["left"]["intrinsics"]["k1"]
  cfg[:intrinsics][:k2] = data["left"]["intrinsics"]["k2"]
  cfg
end


# add AprilTag sightings from this pose
function addApriltags!(fg, pssym, posetags; bnoise=0.1, rnoise=0.1, lmtype=Point2, fcttype=Pose2Pose2, DAerrors=0.0 )
  @show currtags = ls(fg)[2]
  for lmid in keys(posetags)
    @show lmsym = Symbol("l$lmid")
    if !(lmsym in currtags)
      info("adding node $lmsym")
      addNode!(fg, lmsym, lmtype)
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
                         diagm([0.1;0.1;0.01].^2)) )
    end
    if rand() > DAerrors
      # regular single hypothesis
      addFactor!(fg, [pssym; lmsym], ppbr, autoinit=false)
    else
      # artificial errors to data association occur
      info("Forcing bad data association with $lmsym")
      xx,ll = ls(fg)
      @show ll2 = setdiff(ll, [lmsym])
      @show daidx = round(Int, (length(ll2)-1)*rand()+1)
      @show rda = ll2[daidx]
      addFactor!(fg, [pssym; lmsym; rda], ppbr, autoinit=false, multihypo=[1.0;0.5;0.5])
    end
  end
  nothing
end

function addnextpose!(fg, prev_psid, new_psid, pose_tag_bag; lmtype=Point2, odotype=Pose2Pose2, fcttype=Pose2Pose2, DAerrors=0.0)
  prev_pssym = Symbol("x$(prev_psid)")
  new_pssym = Symbol("x$(new_psid)")
  # first pose with zero prior
  if odotype == Pose2Pose2
    addNode!(fg, new_pssym, Pose2)
    addFactor!(fg, [prev_pssym; new_pssym], Pose2Pose2(MvNormal(zeros(3),diagm([0.4;0.1;0.4].^2))))
  elseif odotype == VelPose2VelPose2
    addNode!(fg, new_pssym, DynPose2(ut=round(Int, 200_000*(new_psid))))
    addFactor!(fg, [prev_pssym; new_pssym], VelPose2VelPose2(MvNormal(zeros(3),diagm([0.4;0.07;0.1].^2)),
                                                             MvNormal(zeros(2),diagm([0.2;0.2].^2))))
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
    save(imgsavedir*"/tags/img_$(psid).jpg", IMGS[psid+1])
  end
  # psid = 1
  # img = load(datafolder*"$(imgfolder)/$(camlookup[psid])")

  # free the detector memory
  freeDetector!(detector)

  return IMGS, TAGS
end






#
