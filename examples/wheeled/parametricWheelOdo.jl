using TransformUtils


# in-place x, speed vc, steering alpha, dt
function uteOdom(x::Array{Float64,1}, vc::Float64, alpha::Float64, dt::Float64)
  L = 2.83; H=076; b=0.5;a=3.78;
  d = [(vc*cos(x[3])-vc/L*tan(x[3])*(a*sin(x[3])+b*cos(x[3])));
       (vc*sin(x[3])+vc/L*tan(x[3])*(a*cos(x[3])-b*sin(x[3])));
        vc/L*tan(alpha)]
  x += dt*d
  x[3] = wrapRad(x[3])
  return x
end

function allOdo(sensors::Array{Float64,2})
  odo = zeros(size(sensors,1),3)
  T0 = 0.0
  for i in 2:size(sensors,1)
    odo[i,:]=uteOdom(vec(odo[i-1,:]),sensors[i,2],sensors[i,3],sensors[i,1]-T0)
    T0 = sensors[i,1]
  end

  return odo
end

vc(v::Float64, alpha::Float64; L=2.80381, H=0.828329 ) = v./(1.0-tan(alpha)*H/L)
ve(v::Float64, alpha::Float64; L=2.80381, H=0.828329 ) = v.*(1.0-tan(alpha).*H/L)
dPhi(v::Float64, alpha::Float64; L=2.80381) = 1.0*v*tan(alpha)/L

function compensateRawsensors(drs::Array{Float64,1};
                    L=2.80381, H=0.828329, whlsf=0.94, strsf=1.0199, strbi=0.00159)
    return whlsf*drs[2], strsf*drs[3]+strbi
end

function uteOdomEasy(x::Array{Float64,1}, whlspd::Float64, strangl::Float64, dt::Float64; L=2.80381, H=0.828329)
  wTb = SE2(x)
  v = vc(whlspd,strangl,L=L,H=H)
  dph = dPhi(v,strangl,L=L)
  #dph = v*tan(strangl)
  bTnb = SE2(dt*[v;0.0;dph])
  wTnb = wTb*bTnb
  return vec(se2vee(wTnb))
end

# perform integration over all data, sensors is rows of data, by columns of sensors
# L=2.83, H=0.76, b=0.5, a=3.78
# x = anealfind(x) many times
# julia> x
# 5-element Array{Float64,1}:
 # 0.934965
 # 0.00159147
 # 2.80381
 # 0.828329
 # 1.0199
function allOdoEasy(sensors::Array{Float64,2};
                    L=2.80381, H=0.828329, whlsf=0.94, strsf=1.0199, strbi=0.00159)
  #msensors = [sensors[:,1]';x[1]*sensors[:,2]';x[5]*sensors[:,3]'+x[2]]';
  odo = zeros(size(sensors,1),3)
  for i in 2:size(sensors,1)
    whlspd, strang = compensateRawsensors(vec(sensors[i,:]), L=L,H=H, whlsf=whlsf, strbi=strbi,strsf=strsf)
    odo[i,:] = uteOdomEasy(vec(odo[i-1,:]), whlspd, strang, (sensors[i,1]-sensors[i-1,1]), L=L, H=H )
  end

  return odo
end
