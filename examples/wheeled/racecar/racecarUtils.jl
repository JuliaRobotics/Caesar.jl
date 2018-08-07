



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
function addApriltags!(fg, pssym, posetags; bnoise=0.1, rnoise=0.1, lmtype=Point2 )
  currtags = ls(fg)[2]
  for lmid in keys(posetags)
    @show lmsym = Symbol("l$lmid")
    if !(lmsym in currtags)
      addNode!(fg, lmsym, lmtype)
    end
    ppbr = nothing
    if lmtype == Point2
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
      ppbr = Pose2Pose2(
                MvNormal([dx;
                          dy;
                          dth],
                         diagm([0.3;0.1;0.1].^2)) )
    end
    addFactor!(fg, [pssym; lmsym], ppbr, autoinit=false)
  end
  nothing
end

function addnextpose!(fg, prev_psid, new_psid, pose_tag_bag; lmtype=Point2, odotype=Pose2Pose2)
  prev_pssym = Symbol("x$(prev_psid)")
  new_pssym = Symbol("x$(new_psid)")
  # first pose with zero prior
  if odotype == Pose2Pose2
    addNode!(fg, new_pssym, Pose2)
    addFactor!(fg, [prev_pssym; new_pssym], Pose2Pose2(MvNormal(zeros(3),diagm([0.4;0.1;0.4].^2))))
  elseif odotype == VelPose2VelPose2
    addNode!(fg, new_pssym, DynPose2(ut=round(Int, 1000_000*(new_psid/6.0))))
    addFactor!(fg, [prev_pssym; new_pssym], VelPose2VelPose2(MvNormal(zeros(3),diagm([0.4;0.1;0.4].^2)),
                                                             MvNormal(zeros(2),diagm([0.1;0.01].^2))))
  end

  addApriltags!(fg, new_pssym, pose_tag_bag, lmtype=lmtype)
  new_pssym
end


function drawThickLine!(image, startpoint, endpoint, colour, thickness)
    (row, col) = size(image)
    x1 = startpoint.x
    y1 = startpoint.y
    x2 = endpoint.x
    y2 = endpoint.y

    draw!(image, LineSegment(x1,y1,x2,y2), colour)

    if x1 != x2 && (y2-y1)/(x2-x1) < 1
        for tn = 1:thickness
            i = tn ÷ 2

            x1mi = x1-i
            x1pi = x1+i
            y1mi = y1-i
            y1pi = y1+i
            x2mi = x2-i
            x2pi = x2+i
            y2mi = y2-i
            y2pi = y2+i
            #clip to protect bounds
            (x1mi < 1) && (x1mi = 1)
            (y1mi < 1) && (y1mi = 1)
            (x1pi > col) && (x1pi = col)
            (y1pi > row) && (y1pi = row)
            (x2mi < 1) && (x2mi = 1)
            (y2mi < 1) && (y2mi = 1)
            (x2pi > col) && (x2pi = col)
            (y2pi > row) && (y2pi = row)

            iseven(tn) && draw!(image, LineSegment(x1,y1mi,x2,y2mi), colour)
            isodd(tn)  && draw!(image, LineSegment(x1,y1pi,x2,y2pi), colour)
        end
    else
        for tn = 1:thickness
            i = tn ÷ 2

            x1mi = x1-i
            x1pi = x1+i
            y1mi = y1-i
            y1pi = y1+i
            x2mi = x2-i
            x2pi = x2+i
            y2mi = y2-i
            y2pi = y2+i
            #clip to protect bounds
            (x1mi < 1) && (x1mi = 1)
            (y1mi < 1) && (y1mi = 1)
            (x1pi > col) && (x1pi = col)
            (y1pi > row) && (y1pi = row)
            (x2mi < 1) && (x2mi = 1)
            (y2mi < 1) && (y2mi = 1)
            (x2pi > col) && (x2pi = col)
            (y2pi > row) && (y2pi = row)

            iseven(tn) && draw!(image, LineSegment(x1mi,y1,x2mi,y2), colour)
            isodd(tn)  && draw!(image, LineSegment(x1pi,y1,x2pi,y2), colour)
        end
    end
end






function drawTagLine!(imgl, tag_detection)
  # naive
  # ch, cw = round(Int, height/2), round(Int, width/2)
  cw, ch = 330.4173, 196.32587  # from ZED driver config
  focal = 340.97913
  imheight = 376

  tagsize = 0.172

  bearing0 = tag_detection[:bearing][1]

  # convert bearing to position on image
  tag_u_coord = cw + focal*tan(-bearing0)
  tag_u_coord = round(Int, tag_u_coord)

  c1 = Point(tag_u_coord, 1)
  c2 = Point(tag_u_coord, imheight)
  drawThickLine!(imgl, c1, c2, RGB{N0f8}(0.8,0.0,0.4), 10)

  nothing
end






function plotPose2Vels(fgl::FactorGraph, sym::Symbol; coord=nothing)
  X = getVertKDE(fgl, sym)
  px = plotKDE(X, dims=[4], title="Velx")
  coord != nothing ? (px.coord = coord) : nothing
  py = plotKDE(X, dims=[5], title="Vely")
  coord != nothing ? (py.coord = coord) : nothing
  hstack(px, py)
end




#
