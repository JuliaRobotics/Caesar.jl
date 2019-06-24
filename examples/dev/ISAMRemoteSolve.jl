
ENABLISAMREF = false
cl = Union{}
clr = Union{}

function addOdoRemote(frm,to,DX,cov)
  if ENABLISAMREF
    println(cl, "ODOMETRY $(frm) $(to) $(DX[1]) $(DX[2]) $(DX[3]) $(cov[1,1]) $(cov[1,2]) $(cov[1,3]) $(cov[2,2]) $(cov[2,3]) $(cov[3,3])")
  end
  nothing
end

function addLandmMeasRemote(frm,to,DX,cov)
  if !ENABLISAMREF return nothing end
  println(cl, "LANDMARK $(frm) $(to) $(DX[1]) $(DX[2]) $(cov[1,1]) $(cov[1,2]) $(cov[2,2])")
  nothing
end

function requestAllPosesLandmsRemote(fg::G) where G <: AbstractDFG
  if !ENABLISAMREF return nothing end
  dict = Dict{Int,Array{Float64,1}}()
  xx,ll=ls(fg)
  for x in xx
    println("GETVAL $(fg.IDs[x])")
    println(cl, "GETVAL $(fg.IDs[x])")
  end
  for l in ll
    println("GETVAL $(fg.IDs[l])")
    println(cl, "GETVAL $(fg.IDs[l])")
  end
  for line in readlines(clr)
    @show line
    spl = split(line)
    idx = parse(Int, spl[2])
    arr = Float64[]
    for i in 3:length(spl)
      push!(arr, parse(Float64, spl[i]))
    end
    dict[idx] = arr
  end
  println("quit")
  println(cl,"quit")
  close(cl)
  @show size(dict)
  return dict
end

function doISAMSolve(d,f;toT=Inf, savejld=false, retfg=false, MM=MM)
  #nc -l 2389 | python tcpisam.py | grep "POSE\|LNDM" | nc -l 2390
  global ENABLISAMREF, cl, clr
  ENABLISAMREF = true
  cl = connect("localhost", 2389)
  clr = connect("localhost", 2390)

  fgu = emptyFactorGraph()
  appendFactorGraph!(fgu, d, f, toT=toT,lcmode=:unimodal,MM=MM);
  println(cl, "BATCHSOLVE ")
  println("BATCHSOLVE")
  gtvals = requestAllPosesLandmsRemote(fgu)
  close(clr)
  if savejld
    jldopen("fgWithISAMref.jld", "w") do file
       write(file, "isamdict", gtvals)
       write(file, "fgu", fgu)
    end
  end
  ENABLISAMREF = false

  return !retfg ? gtvals : gtvals, fgu
end

function computeGraphResiduals(fgl::G, dict::Dict{Int,Array{Float64,1}}) where G <: AbstractDFG
  X,Y,Th,LB = get2DPoseMax(fgl)
  XL,YL,ThL,LBL = get2DLandmMax(fgl)

  resid = Float64[]
  residTh = Float64[]

  for idx in 1:length(LB)
    pred = dict[fgl.IDs[LB[idx]]]
    push!(resid, norm([X[idx];Y[idx]]-pred[1:2]))
    push!(residTh, norm(Th[idx]-pred[3]))
  end
  #for lidx in 1:length(LBL)
  #  pred = dict[fgl.IDs[LBL[lidx]]]
  #  push!(resid, norm([XL[lidx];YL[lidx]]-pred))
  #end
  return resid, residTh, LB #[LB;LBL]
end

function drawUnimodalPoses(LB, Xpp, Ypp, Thpp;c="blue",lw=2pt,prtstb=true, lbls=true)
  psplt = Union{}
  if lbls
    psplt = Gadfly.plot(
    Gadfly.layer(x=Xpp,y=Ypp,label=LB,Geom.path(), Theme(line_width=lw, default_color=parse(Colorant,c)), Geom.label)
    )
  else
    psplt = Gadfly.plot(
    Gadfly.layer(x=Xpp,y=Ypp,Geom.path(), Theme(line_width=lw, default_color=parse(Colorant,c)))
    )
  end
  if prtstb
    stbPrtLineLayers!(psplt, Xpp, Ypp, Thpp)
  end

  psplt
end

function drawUnimodalLandm(LBL, Xpp, Ypp;c="red", lbls=true)
  psplt = Union{}
  if lbls
    psplt = Gadfly.plot(
    Gadfly.layer(x=Xpp,y=Ypp, label=LBL, Geom.point, Theme(line_width=2pt, default_color=parse(Colorant,c)), Geom.label)
    )
  else
    psplt = Gadfly.plot(
    Gadfly.layer(x=Xpp,y=Ypp, Geom.point, Theme(line_width=2pt, default_color=parse(Colorant,c)))
    )
  end
  return psplt
end

function convertISAMDictArr(fgu::G, isamdict::Dict{Int,Array{Float64,1}}) where G <: AbstractDFG
  X=Float64[];Y=Float64[];Th=Float64[];
  Xl=Float64[];Yl=Float64[];

  xx,ll=ls(fgu)
  # ll = removeKeysFromArr(fgl, collect(keys(MM)), ll);

  for x in xx
    val = isamdict[fgu.IDs[x]]
    push!(X,val[1])
    push!(Y,val[2])
    push!(Th,val[3])
  end
  for l in ll
    val = isamdict[fgu.IDs[l]]
    push!(Xl,val[1])
    push!(Yl,val[2])
  end
  return xx, X, Y, Th, ll, Xl, Yl
end

function drawUniPosesLandm(fgu::G, isamdict::Dict{Int,Array{Float64,1}}) where G <: AbstractDFG

  xx, X, Y, Th, ll, Xl, Yl = convertISAMDictArr(fgu, isamdict)

  p = drawUnimodalPoses(xx,X,Y,Th)
  pl = drawUnimodalLandm(ll,Xl,Yl)

  for l in pl.layers
    push!(p.layers, l)
  end
  return p
end

function drawCompPosesLandm(fgl::G,isamdict::Dict{Int,Array{Float64,1}},fgu::G;
                            lbls=true, drawhist=false, drawunilm=true) where G <: AbstractDFG
  p=drawPosesLandms(fgl,lbls=lbls,drawhist=drawhist, sho)

  xx, X, Y, Th, ll, Xl, Yl = convertISAMDictArr(fgu, isamdict)
  ptp = drawUnimodalPoses(xx,X,Y,Th,c="magenta",lw=1pt,prtstb=false, lbls=lbls)

  for l in ptp.layers
    push!(p.layers, l)
  end
  if drawunilm
    ptl = drawUnimodalLandm(ll,Xl,Yl,c="magenta", lbls=lbls)
    for l in ptl.layers
      push!(p.layers, l)
    end
  end

  return p
end


# @async begin
#    server = listen(2390)
#    while true
#      sock = accept(server)
#      @async while isopen(sock)
#        @show readline(sock)
#      end
#    end
#  end
