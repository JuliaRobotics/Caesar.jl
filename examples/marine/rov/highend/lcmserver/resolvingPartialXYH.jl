# Resolving Partial XYH constraints

using TransformUtils
using CoordinateTransformations
using DrakeVisualizer
import DrakeVisualizer: Triad

import Base: convert
using Base: Test

import Rotations as _Rot


# random testing and converstion to be moved out

function convert{T <: CoordinateTransformations.AffineMap}(::Type{T}, x::SE3)
  q = convert(TransformUtils.Quaternion, x.R)
  Translation(x.t...) ∘ LinearMap( _Rot.QuatRotation(q.s, q.v...) )
end
function convert{T <: CoordinateTransformations.AffineMap{_Rot.QuatRotation{Float64}}}(::Type{SE3}, x::T)
  SE3(x.v[1:3], TransformUtils.Quaternion(x.m.w, [x.m.x,x.m.y,x.m.z]) )
end

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


#
