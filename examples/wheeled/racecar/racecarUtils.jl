



function loadConfig()
  cfg = Dict{Symbol,Any}()
  data =   YAML.load(open(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cam_cal.yml")))
  bRc = eval(parse("["*data["extrinsics"]["bRc"][1]*"]"))
  cfg[:bRc] = bRc
  cfg[:intrinsics] = Dict{Symbol,Any}()
  cfg[:intrinsics][:height] = data["intrinsics"]["height"]
  cfg[:intrinsics][:width] = data["intrinsics"]["width"]
  cfg[:intrinsics][:cam_matrix] = data["intrinsics"]["camera_matrix"]
  cfg
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






#
