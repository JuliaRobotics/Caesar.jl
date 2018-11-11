


function drawTagDetection(vis::Visualizer, tagname, Q, T, bTc, bP2t; posename=:test)
  # draw tag triad
  setobject!(vis[currtag], Triad(0.2))
  settransform!(vis[currtag], bTt)

  # draw ray to tag
  v = vis[posename][:lines][tagname]
  geometry = PointCloud(
  GeometryTypes.Point.([bTt.translation[1];0],
                       [bTt.translation[2];0],
                       [0.0;0])
  )
  setobject!(v, LineSegments(geometry, LineBasicMaterial()))
  settransform!(v, Translation(0.0,0,0))

  # draw orientation tail for tag
  v = vis[posename][:orient][tagname]
  geometry = PointCloud(
  GeometryTypes.Point.([0.1;0],
                       [0.0;0],
                       [0.0;0])
  )
  setobject!(v, LineSegments(geometry, LineBasicMaterial()))
  settransform!(v, bP2t) #
  nothing
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
  @warn "Using hard-coded camera calibration parameters in drawTagLine!"
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




# tan(bear) = u/f




# L = X + DX  =>  X\L = DX

function imageFactor(fg, fsym, im, cfg)
  img = Gray.(im)

  zmeas = getData(getVert(fg, fsym,nt=:fnc)).fnc.usrfnc!.Zpose.z.μ

  # @show IIF.lsf(fg, fsym)
  XX, LL = (KDE.getKDEMax.(IIF.getVertKDE.(fg, IIF.lsf(fg, fsym)))...)
  @show xyt = se2vee(SE2(XX[1:3]) \ SE2([LL[1:2];0.0]))
  bear= TU.wrapRad(atan2(-xyt[2],xyt[1]) - XX[3])
  # bear = 0.0
  focal= cfg[:intrinsics][:fx]
  cu = cfg[:intrinsics][:cx]
  #
  @show bu = tan(bear)*focal  +  cu
  intens = pdf.(Distributions.Normal(bu,30.0), collect(1:size(img,2)))
  intens ./= maximum(intens)

  zbear= TU.wrapRad(atan2(-zmeas[2],zmeas[1]) - 0.0)

  @show zbu = tan(zbear)*focal  +  cu
  zintens = pdf.(Distributions.Normal(zbu,30.0), collect(1:size(img,2)))
  zintens ./= maximum(zintens)


  for i in 1:round(Int,0.3*size(img, 1))
    img[i, :] .*= intens
  end
  for i in round(Int,0.3*size(img, 1)):size(img,1)
    img[i, :] .*= zintens
  end

  img
end


function drawAllBearingImgs(fg, IMGS, cfg)
  BRIMGS = Dict{Symbol,Dict{Symbol, Any}}()
  XX, LL = ls(fg)

  idx = 0
  for x in XX
    idx += 1
    BRIMGS[x] = Dict{Symbol,Any}()
    for l in LL
      try
        fsym = Symbol("$(x)$(l)f1")
        im = imageFactor(fg, fsym, IMGS[idx], cfg)
        BRIMGS[x][l] = im
      catch Er
        #
      end
    end
  end
  return BRIMGS
end


function genGifLandm(fg, sym, IMGS; show=true, delay=50, dir="")
  BIM = drawAllBearingImgs(fg, IMGS, cfg)
  TEMP = []
  for ss in split.(string.(ls(fg,sym)),string(sym))
    if haskey(BIM, Symbol(ss[1]))
      if haskey(BIM[Symbol(ss[1])], sym)
       push!(TEMP, BIM[Symbol(ss[1])][sym])
      end
    end
  end

  # mkdir("/tmp/animate_tags")
  # Base.rm("/tmp/animate_tags")
  run(`rm -fr /tmp/animate_tags`)
  mkdir("/tmp/animate_tags")
  for i in 1:length(TEMP)
    save("/tmp/animate_tags/im$(i).png", TEMP[i])
  end

  run(`convert -delay $(delay) /tmp/animate_tags/im*.png $(dir)$(sym).gif`)

  !show ? nothing : @async run(`eog $(sym).gif`)
  return "$(sym).gif"
end


function getAllLandmGifs(fg, IMGS; show=false, dir="")
  info("Dropping gifs here $(Base.pwd())")
  for l in ls(fg)[2]
    genGifLandm(fg, l, IMGS, show=show, dir=dir)
  end
  nothing
end

function plotPoseVelAsMax(fg, poserange)

  xx = Symbol.(string.("x",collect(poserange)))

  len = length(xx)
  VV = zeros(len, 2)

  i = 0
  for state in KDE.getKDEMax.(getVertKDE.(fg, xx))
    i += 1
    VV[i,:] = state[4:5]
  end

  Gadfly.plot(
    Gadfly.layer(x=poserange, y=VV[:,1], Geom.line, Theme(default_color=colorant"red")),
    Gadfly.layer(x=poserange, y=VV[:,2], Geom.line, Theme(default_color=colorant"green"))
  )
end







# # save factor graph for later testing and evaluation
# fg, = loadjld(file=resultsdir*"/racecar_fg_x200_presolve.jld2")
# fg, = loadjld(file=resultsdir*"/racecar_fg_??.jld2")
# @time tree = batchSolve!(fg, N=N, drawpdf=true, show=true, recursive=true)
# 0
#
# IIF.savejld(fg, file=resultsdir*"/racecar_fg_final_resolve.jld2")
# # fgr, = loadjld(file=resultsdir*"/racecar_fg_final_resolve.jld2")
# results2csv(fg; dir=resultsdir, filename="results_resolve.csv")
#
#
# # pl = plotKDE(fg, :x1, levels=1, dims=[1;2]);
#
#
#
# #,xmin=-3,xmax=6,ymin=-5,ymax=2);
# Gadfly.push_theme(:default)
# pl = drawPosesLandms(fg, spscale=0.1, drawhist=false, meanmax=:max);
# # Gadfly.set(:default_theme)
# Gadfly.draw(PNG(joinpath(resultsdir,"images","final.png"),15cm, 10cm),pl);
# Gadfly.draw(SVG(joinpath(resultsdir,"images","final.svg"),15cm, 10cm),pl);
#
# # pl = drawPosesLandms(fg, spscale=0.1, meanmax=:mean) # ,xmin=-3,xmax=3,ymin=-2,ymax=2);
# # Gadfly.draw(PNG(joinpath(resultsdir,"hist_final.png"),15cm, 10cm),pl)
#
# 0




# debugging

# using Profile
#
# using Logging
# global_logger(NullLogger())
# @profile println("test")
# Profile.clear()
# @profile tree1 = IIF.wipeBuildNewTree!(fg)
#
# Juno.profiler()


# vars = lsRear(fg, 5)
#
#
# num_edges(fg.g)
#
# subgraphFromVerts(fg, vars)
#
# ensureAllInitialized!(fg)
# isInitialized(fg, :x220)
#
# ls(fg, :l14)




#
