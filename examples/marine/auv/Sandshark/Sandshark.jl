# build factor graph from sandshark data
addprocs(7)
using Caesar
using RoME
using IncrementalInference, KernelDensityEstimate
using MAT
using JLD, HDF5
using Gadfly

vars = matread("/home/dehann/data/sandshark/cbf.mat");

len = length(vars["imu_gyr_x"]);

accs = zeros(len,3);
gyrs = zeros(len,3);

lbl = ["x";"y";"z"]
for i in 1:3 accs[:,i] = vars["imu_acc_$(lbl[i])"][:]; end
for i in 1:3 gyrs[:,i] = vars["imu_gyr_$(lbl[i])"][:]; end
imutime = vars["imu_time"];

nativeOdo = zeros(length(vars["nav_time"][:]), 5);
nativeOdo[:,1] = vars["nav_time"][:];
nativeOdo[:,2] = 0.68*vars["nav_speed"][:];
nativeOdo[:,3] = vars["nav_roll"][:]*pi/180.0;
nativeOdo[:,4] = vars["nav_pitch"][:]*pi/180.0;
nativeOdo[:,5] = (vars["nav_yaw"][:]-14.72)*pi/180.0; # assumed to be heading

nativeOdo[:,1] -= vars["nav_time"][1]
nativeOdo[:,1] *= 1e-9

for i in 1:size(nativeOdo,1)
  nativeOdo[i,5] = wrapRad( (2pi-nativeOdo[i,5])-3.0*pi/2.0 )
end

nativeOdo[2:end,5] = diff(nativeOdo[:,5])
nativeOdo[1,5] = 0
nativeOdo[nativeOdo[:,5] .> 2pi-0.1, 5] -= 2pi
nativeOdo[nativeOdo[:,5] .< -2pi+0.1, 5] += 2pi

nativeMeas = zeros(length(vars["CBF_MF_time"]),5);
nativeMeas[:,1] = (vars["CBF_MF_time"][:]-(vars["nav_time"][1]*1e-9));
# quality estimate of acoustic measurement
nativeMeas[:,2] = vars["MF_std"]
# elevation' and azimuth -- TODO remove roll, pitch from body relative azimuth
nativeMeas[:,3:4] = vars["CBF_max"][:,1:2]
# argmax range
for i in 1:length(vars["CBF_MF_time"])
  nativeMeas[i,5] =findmax(sum(abs(vars["MF_prob"][:,:,i]),2))[2]
end
# nativeMeas[:,5] = findmax(sum(abs(vars["MF_prob"]),2),1)[2][:]
nativeMeas[:,5] -= 400.0
nativeMeas[:,5] *= 1500.0/37500

@parallel for i in  1:905
  pls = []
  for j in 1:4
    push!(pls, plot(y=(abs(vars["MF_prob"][:,j,i])),Geom.line,Coord.Cartesian(ymax=0.2)));
  end
  push!(pls, plot(y=sum(abs(vars["MF_prob"][:,:,i]),2),Geom.line,Coord.Cartesian(ymax=0.2)));
  pl = vstack(pls...)
  draw(PNG("/home/dehann/Desktop/here/MF_prob_$(i).png",10cm,35cm),pl);
end


# integrate native measurements for dead reckoning
# Dxi_k1 is previous k-1 position in pose i frame, x_{k-1}=xk1
function sandsharkDeadReckoning2D(dt, x, speed, pitch, dyaw)
  Dxi_k1 = SE2(x)
  v = speed*cos(pitch)
  D = SE2([0.0;0.0;dyaw])*SE2(dt*[v;0.0;0.0])
  Dxi_k = Dxi_k1*D
  return vec(se2vee(Dxi_k))
end


function deltaOdom(xprev::Array{Float64,1}, xnow::Array{Float64,1})
  xk1 = SE2(xprev)
  xk = SE2(xnow)
  return se2vee((xk1 \ xk))
end

# odo has form odo[1:n,:] = [t, x, y, theta], with n rows of data in time
function advOdoByRules(odo::Array{Float64,2}, meas::Array{Float64,2};
                        distrule=10.0, timerule=30.0, yawrule=pi/6.0, trkfeats=false)
  # DBG = Dict{Int,Dict{Int64,Array{BallTreeDensity,1}}}()
  # DBG = Dict{Int,Dict{Int64,Feature}}()
  DBG = nothing
  cntr = 1

  # bfts0 = Array{Float64,2}() #lsrFeats[1].feats
  instSys = makeGenericInSituSys(zeros(3))
  instSys.T0 = odo[1,1] # in nanoseconds
  fdict = Dict{Int64,Array{Float64,1}}()
  # for f in instSys.trackers
  #   # if length(f[2].lastz) < 3
  #   #   error("HEY!! $(f[1])")
  #   # end
  #   fdict[f[2].id] = f[2].lastz
  # end
  # instSys.FeatAssc[instSys.poseid] = fdict
  for i in 1:size(odo,1)
    dt = (odo[i,1]-instSys.T0)
    bTbm = SE2(instSys.x)
    # instSys.x is the real time odometry estimate
    instSys.x = sandsharkDeadReckoning2D(dt, instSys.x, odo[i,2], odo[i,4],odo[i,5])
    # bTbp = SE2(instSys.x)

    trigmeas = false
    if cntr <= size(meas,1)
      if odo[i,1] >= meas[cntr,1]
        # TODO -- relax to include at least range on large azimuth
        if meas[cntr,2] < 5.0 &&  (meas[cntr,4] < 95.0 || 265.0 < meas[cntr,4])
          trigmeas = true
        end
        cntr+=1
      end
    end
  #   # subsample motion tracks for constructing the factor graph SLAM solution
    if poseTrigAndAdd!(instSys, (odo[i,1]), distrule, timerule, yawrule, auxtrig=trigmeas)
      fdict = Dict{Int64,Array{Float64,1}}()
      if trigmeas
        bearing = wrapRad(meas[cntr-1,4]*pi/180.0)
        fdict[1] = [meas[cntr-1,5];bearing;0.0]
      end
      instSys.FeatAssc[instSys.poseid] = fdict
    end
  #   # advance previous time point
    instSys.T0 = odo[i,1]
  end
  return instSys.dOdo, instSys.FeatAssc, DBG
end


XX = advOdoByRules(nativeOdo, nativeMeas);
XX[2][1] = Dict{Int64,Array{Float64,1}}()

progressExamplePlot(XX[1],XX[2])

MM = Dict{Int, Int}()

fg = emptyFactorGraph();
idx = appendFactorGraph!(fg, XX[1], XX[2], lcmode=:unimodal, MM=MM);

tree = prepBatchTree!(fg, ordering=:qr,drawpdf=true);
inferOverTree!(fg,tree, N=100);

isamdict, fgu = doISAMSolve(XX[1],XX[2], savejld=false, MM=MM, retfg=true)
isamdict[1] = zeros(3)
# @save "odoSandsharkISAM1.jld" isamdict fgu
@load "odoSandsharkISAM1.jld"

@save "sandsharkSolve01.jld" fg nativeOdo nativeMeas

pl = drawCompPosesLandm(fg, isamdict, fgu, lbls=false)
using Colors, Cairo
draw(PDF("ss01.pdf",20cm,40cm),pl)

function testSandsharkOdo(;iters::Int=10)
  odo = zeros(iters, 5)
  dt = 0.1
  odo[:,1] = collect(1:iters)*dt
  odo[:,2] = 1.0
  odo[4:end,5] = pi/4
  odo[8:end,5] = pi/2.0

  prevpose = zeros(3)
  prevx = zeros(3)
  X = Array{Float64,2}(iters,3)
  for i in 1:iters
    x = sandsharkDeadReckoning2D(dt, prevx, odo[i,2], odo[i,4], odo[i,5])
    X[i,:] = deepcopy(x)
    prevx = x
    # do pose trigger test
      # new pose
  end

  return X
end

# X = testSandsharkOdoReal(nativeOdo,iters=10000);
# plot(x=X[:,1],y=X[:,2], Geom.path())
function testSandsharkOdoReal(odo;iters::Int=100)

  Tprev = odo[1,1]

  prevx = zeros(3)
  X = Array{Float64,2}(iters,3)
  for i in 1:iters
    dt = (odo[i,1] - Tprev)
    x = sandsharkDeadReckoning2D(dt, prevx, odo[i,2], odo[i,4], odo[i,5])
    X[i,:] = deepcopy(x)
    prevx = x
    # do pose trigger test
      # new pose
    Tprev = odo[i,1]
  end

  return X
end
