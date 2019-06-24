## resolve the ground truth trajectory from ordinary differential equations

# ddy = g
# dx/dt = Fx + Gu
# [ddy;   = [0  0][dy  + [1][g=-10]
#   dy ]    [1  0]  y]   [0]
function falling(du,u,p,t)
  du[1] = 0    # drag
  du[2] = -10  # gravity
  du[3] = u[1]
  du[4] = u[2]
  nothing
end

u0 = [5,15,-20.0,0.0]
tspan = (0.0,5.0)
prob = ODEProblem(falling,u0,tspan)
sol = solve(prob)
# sol(t)

Plots.plot(sol, vars=(3,4))


## extract the desired measurement information from the physical simulation

tstop = 5.0
nposes = 10
ts = range(0, stop=tstop, length=nposes)
XY = zeros(2,nposes)

for i in 1:nposes
  t = ts[i]
  XY[:,i] = sol(t)[3:4]
end

gx = (t)->sol(t)[3]
gy = (t)->sol(t)[4]




# Calculate the true radar range and angle

ranges = sqrt.(vec(sum(XY.^2, dims=1)))
angles = atan.(XY[2,:], XY[1,:])


XYdense = [gx.(0:0.01:tstop) gy.(0:0.01:tstop)]';
randense = sqrt.(vec(sum(XYdense.^2, dims=1)))
angdense = atan.(XYdense[2,:], XYdense[1,:])
