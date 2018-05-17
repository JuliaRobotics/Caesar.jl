# Resolving Partial XYH constraints

using TransformUtils
using CoordinateTransformations
using DrakeVisualizer
import DrakeVisualizer: Triad

using Base: Test




# random testing and converstion to be moved out


tq = SE3(zeros(3),Quaternion(0.0,[1.0, 0,0]));
q1 = convert(AffineMap, tq);
q2 = convert(AffineMap, tq);

q12 = q1 ∘ q2

tq12 = convert(SE3, q12)

@test compare(SE3(0), tq12)

# testing complete, continue with XYH example



# Launch the viewer application if it isn't running already:
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window();
vis = Visualizer();

# okay, do the numbers
dist = 10.0

wTx1 = SE3(zeros(3), TransformUtils.AngleAxis(-pi/2,[1.0,0,0]))
wTx2 = SE3([0,-dist,0], TransformUtils.AngleAxis(-pi/2,[1.0,0,0]))

# x1
setgeometry!(vis[:evalXYH][:x1], Triad());
settransform!(vis[:evalXYH][:x1], convert(AffineMap,wTx1 ) );
# x2
setgeometry!(vis[:evalXYH][:x2], Triad());
settransform!(vis[:evalXYH][:x2], convert(AffineMap,wTx2 ) );

## Look at XYH constraint numbers
# iSAM is doing
# predicted = p2.ominus(p1);
# Eigen::Vector3d predicted3(predicted.x(), predicted.y(), standardRad(predicted.yaw()));
# Eigen::Vector3d err = predicted3 - _measure;
# Pose3d ominus(const Pose3d& b) const {
#     return Pose3d(b.oTw() * wTo());
#   }

x1Tx2 = wTx1\wTx2
predicted = [x1Tx2.t[1:2]..., convert(Euler, x1Tx2.R).Y]

@show predicted

@test norm(predicted - dist) < 1e-10



# new local level


# wTx1 = wTx1L * x1LTx1
z = wTx1l * o - wTx2l
o = x1lTx2l #(estimate)
# where wTx1 = wTx1l * x1lTx1 # last T is just roll after pitch (assuming YPR)


# convention
aV = aTb*bV

# 6DOF
wTx1 = SE3(zeros(3), Quaternion(0))
# wTx1 = SE3(zeros(3), TransformUtils.AngleAxis(-pi/2,[1.0,0,0]))
wTx2 = SE3([0,-10,0], TransformUtils.AngleAxis(0.0,[1.0,0,0]))


wTx1l # is already in local level

# 3DOF constaint
# we want this!!
        = wTx2l ominus wTx1l  # Lu & Milios
        = wTx1l \ wTx2l       # who cares
x1lTx2l = x1lTw * wTx2l


wTx1l =
# local-level frames (yay for YPR)
# force z to 0 so that the resulting difference (ominus) has no difference in z
# and is effectively a rank 3 constraint
wTx1l = SE3([wTx1.x, wTx1.y, 0.0], Euler( 0.0, 0.0, wTx1.Yaw) )
wTx2l = SE3([wTx2.x, wTx2.y, 0.0], Euler( 0.0, 0.0, wTx2.Yaw) )

# residual
x1ltilde_T_x1lhat = δ = pred \ meas = meas.ominus( pred )
                      =
# pred \ meas = noise ⊕ error

δ = meas    \- pred
  = [x,y,Θ] \- [xhat, yhat, Θhat]

# measurement is message with [x, y, Θ] in local level of x1 including its heading
x1lRx1 = SE3(zeros(3), Euler( wTx1.Roll, wTx1.Pitch, 0.0) )  # is only roll and pitch
wTx1 = wTx1l * x1lRx1
wTx1 = wtx1ln * wRx1l * x1lRx1p * x1pRx1r
# wTx1 = translation * yaw * pitch * roll
# wTx1 = wTx1l             * x1lRx1

wR == x1lnR


# iSAM.YPR == CoordinateTransformations.RotXYZ == TransformUtils.Euler(R,P,Y)
# on Euler
convert(  )


#
