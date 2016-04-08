# module VictoriaParkTypes
#
# using Gadfly
#
# export LaserFeatures,
#         uteOdom,
#         uteOdomEasy,
#         allOdoEasy,
#         plotParVicPrk,
#         advOdoByRules,
#         compensateRawDRS,
#         plotPoseDict,
#         progressExamplePlot,
#         rotateFeatsToWorld,
#         getFeatsAtT
#
# export R, SE2, se2vee, wrapRad

type LaserFeatures
  t::Float64
  feats::Array{Float64,2}
end

# jldopen("datasets/VicPrk.jld","w") do file
#    addrequire(file, VictoriaParkTypes)
#    write(file, "LsrFeats",LsrFeats)
#    write(file, "DRS", DRS)
#    write(file, "GPS", GPS)
#    write(file, "d", d)
#    write(file, "f", f)
# end
# L=2.83, H=0.76, b=0.5, a=3.78
# x = anealfind(x) many times
# julia> x
# 5-element Array{Float64,1}:
 # 0.934965
 # 0.00159147
 # 2.80381
 # 0.828329
 # 1.0199



 function wrapRad(th::Float64)
   if th >= pi
     th -= 2.0*pi
   end
   if th < -pi
     th += 2.0*pi
   end
   return th
 end

 R(th) = [[cos(th);-sin(th)]';[sin(th);cos(th)]'];

 function SE2(X::Array{Float64,1})
     T = eye(3)
     T[1:2,1:2] = R(X[3])
     T[1,3] = X[1]
     T[2,3] = X[2]
     return T
 end

 function se2vee(T::Array{Float64,2})
     retval = zeros(3,1)
     retval[1:2,1] = T[1:2,3]
     retval[3,1] = wrapRad(atan2(-T[1,2], T[1,1]))
     return retval
 end



function fetchVecML()
  client = connect(4013)
  sleep(0.05)
  res = readline(client)
  close(client)
  ss = split(res,'@')
  @show ss[1], ss[2], ss[3]
  i = parse(Int64, (ss[1]))
  t = parse(Float64,(ss[2]))
  pts = ss[3]
  while client.status != 6 sleep(0.01) end
  sleep(0.05)
  pts = readdlm(IOBuffer(pts),',')
  @show r,c = size(pts)
  if c == 0
      return i, LaserFeatures(t, zeros(3,0))
  end
  return i, LaserFeatures(t, reshape(pts, 3, floor(Int,c/3)))
end


function uteOdom(x::Array{Float64,1}, vc::Float64, alpha::Float64, dt::Float64)
  L = 2.83; H=076; b=0.5;a=3.78;
  d = [(vc*cos(x[3])-vc/L*tan(x[3])*(a*sin(x[3])+b*cos(x[3])));
       (vc*sin(x[3])+vc/L*tan(x[3])*(a*cos(x[3])-b*sin(x[3])));
        vc/L*tan(alpha)]
  x += dt*d
  x[3] = wrapRad(x[3])
  return x
end

function allOdo(DRS::Array{Float64,2})
  odo = zeros(size(DRS,1),3)
  T0 = 0.0
  for i in 2:size(DRS,1)
    odo[i,:]=uteOdom(vec(odo[i-1,:]),DRS[i,2],DRS[i,3],DRS[i,1]-T0)
    T0 = DRS[i,1]
  end

  return odo
end

vc(v::Float64, alpha::Float64; L=2.80381, H=0.828329 ) = v./(1.0-tan(alpha)*H/L)
ve(v::Float64, alpha::Float64; L=2.80381, H=0.828329 ) = v.*(1.0-tan(alpha).*H/L)
dPhi(v::Float64, alpha::Float64; L=2.80381) = 1.0*v*tan(alpha)/L

function compensateRawDRS(drs::Array{Float64,1};
                    L=2.80381, H=0.828329, whlsf=0.94, strsf=1.0199, strbi=0.00159)
    return whlsf*drs[2], strsf*drs[3]+strbi
end

function uteOdomEasy(x::Array{Float64,1}, whlspd::Float64, strangl::Float64, dt::Float64; L=2.80381, H=0.828329)
  pose = SE2(x)
  v = vc(whlspd,strangl,L=L,H=H)
  dph = dPhi(v,strangl,L=L)
  #dph = v*tan(strangl)
  D = SE2(dt*[v;0.0;dph])
  pose = pose*D
  return vec(se2vee(pose))
end

function allOdoEasy(DRS::Array{Float64,2};
                    L=2.80381, H=0.828329, whlsf=0.94, strsf=1.0199, strbi=0.00159)
  #mDRS = [DRS[:,1]';x[1]*DRS[:,2]';x[5]*DRS[:,3]'+x[2]]';
  odo = zeros(size(DRS,1),3)
  for i in 2:size(DRS,1)
    whlspd, strang = compensateRawDRS(vec(DRS[i,:]), L=L,H=H, whlsf=whlsf, strbi=strbi,strsf=strsf)
    odo[i,:] = uteOdomEasy(vec(odo[i-1,:]), whlspd, strang, (DRS[i,1]-DRS[i-1,1]), L=L, H=H )
  end

  return odo
end

function getFeatsAtT(lsrd, T; prev=1)
    if T == 0
        return 1, 0.0
    end
    for i in prev:length(lsrd)
        if lsrd[i].t > T
            return i-1, lsrd[i-1].t
        end
    end
    error("getFeatsAtT -- didnt find anything")
end

# function advOdoByRules(DRS::Array{Float64,2};
#                         distrule=20.0, timerule=30.0, yawrule=pi/3.0)
#   dOdo = Dict{Int64,Array{Float64,1}}()
#   dFtAssc = Dict{Int64, }
#
#   T0 = 0.0
#   poseid = 1
#   x = zeros(3)
#   dOdo[poseid] = [x[1];x[2];x[3];T0;0]
#   wTbk1 = SE2(zeros(3))
#   Tprev = 0.0
#   for i in 1:size(DRS,1)
#     dt = DRS[i,1]-T0
#     whlspd, strang = compensateRawDRS(vec(DRS[i,:]))
#     x=uteOdomEasy(x, whlspd, strang,dt)
#
#     rule = triggerPose(x, zeros(3), DRS[i,1], Tprev, distrule, timerule, yawrule)
#     if rule != 0
#       bk1Tbk = SE2(x)
#       n = [x[1];x[2];x[3];DRS[i,1];rule]
#       poseid +=1
#       dOdo[poseid] = n
#       wTbk1 = wTbk1*bk1Tbk
#       Tprev = DRS[i,1]
#       x[1] = 0.0; x[2]=0.0; x[3]=0.0
#     end
#     T0 = DRS[i,1]
#   end
#
#   return dOdo
# end

function rotateFeatsToWorld(bfts, pose)
    wfts = zeros(size(bfts))
    wfts[3,:] = bfts[3,:]
    for i in 1:size(bfts,2)
        bfT = SE2([0.0;0.0;bfts[2,i]])*SE2([bfts[1,i];0.0;0.0])
        wfts[1:2,i] = vec(se2vee(pose*bfT))[1:2]
    end
    return wfts
end

function plotPoseDict(dOdo::Dict{Int64,Array{Float64,1}};from=1,to=Inf)
    len = length(dOdo)
    if len>to
      len = to
    end
    # T = Array{Float64,1}()
    X = Array{Float64,1}()
    Y = Array{Float64,1}()
    # Th = Array{Float64,1}()

    pose = SE2(zeros(3))
    for i in from:len
        pose = pose*SE2(dOdo[i][1:3])
        x = se2vee(pose)
        # push!(T, odo[i][1])
        push!(X, x[1])
        push!(Y, x[2])
        # push!(Th, x[3])
    end
    plot(x=X, y=Y, Geom.path(), Geom.point)
end


function plotParVicPrk(DRS, x=[0.934965;0.00159147;2.80381;0.828329;1.0199])
   #mDRS = [DRS[:,1]';x[1]*DRS[:,2]';x[5]*DRS[:,3]'+x[2]]';
   modo = allOdoEasy(DRS,whlsf=x[1],strbi=x[2],L=x[3],H=x[4], strsf=x[5]);
   plot(x=modo[:,1],y=modo[:,2],Geom.path(),Guide.xticks(ticks=[-150;250]),Guide.yticks(ticks=[-100;300]))
end

function fullrun(x)
     p=plotParVicPrk(DRS,x)
     draw(PNG("test.png",25cm,25cm),p)
     a = readall(`python floodfill.py`)
     val = parse(Int64,a[1:(end-1)])
     val
 end

x = [1.0;0.002;2.83;0.76;1.0]

function anealfind(x, N=200)
  len = length(x)
  idx = len
  # x = [0.985;0.0023]
  res = fullrun(x)
  for i in 1:N
      idx = (idx)%len+1
      if idx == 1
        epsi = (N/i)*randn()*[0.001;0.0;0.0;0.0;0.0]
      elseif idx == 2
        epsi = (N/i)*randn()*[0.0;0.0001;0.0;0.0;0.0]
      elseif idx == 3
        epsi = (N/i)*randn()*[0.0;0.0;0.001;0.0;0.0]
      elseif idx == 4
        epsi = (N/i)*randn()*[0.0;0.0;0.0;0.001;0.0]
      elseif idx == 5
        epsi = (N/i)*randn()*[0.0;0.0;0.0;0.0;0.001]
      end
      nres = fullrun(x+epsi)
      if nres < res
        res = nres
        @show x += epsi
      end
  end
  x
end

# end
