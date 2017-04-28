# test DrakeVisualizer

# for more low level interaction with the DraekVisualizer/Director, please see
# www.github.com/rdeits/DrakeVisualizer.jl

using Caesar, RoME, CoordinateTransformations, Rotations
using GeometryTypes
using ColorTypes: RGB
# for illustration only
using DrakeVisualizer
import DrakeVisualizer: Triad

vis = startdefaultvisualization(draworigin=true)

setgeometry!(vis[:basics][:triad], Triad())
settransform!(vis[:basics][:triad], Translation(0.0,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/4,0,0,1.)  ))

setgeometry!(vis[:basics][:box], HyperRectangle(Vec(0.,0,0), Vec(1.,1,1)) )
settransform!(vis[:basics][:box], Translation(2.0,0,0) ∘ LinearMap(Rotations.AngleAxis(0.0,0,0,1.)  ))

# and a point cloud

pointcloud = PointCloud([[x, 0, 0.5] for x in linspace(-1, 1)])
pointcloud.channels[:rgb] = [RGB(g, (1-g), 0) for g in linspace(0, 1)]
setgeometry!(vis[:pointcloud], pointcloud)
delete!(vis[:pointcloud])










#
using TransformUtils

setgeometry!(vis[:basics][:origin], Triad())


q = convert(Quaternion, so3([0.3*pi/2,0,0.0]))

qq = Quat(q.s,q.v...)

setgeometry!(vis[:basics][:triad], Triad())
settransform!(vis[:basics][:triad], Translation(0.0,0,0) ∘ LinearMap( qq ))


for val in linspace(0,pi,100)
  q = convert(Quaternion, so3([val,0,0.0]))
  settransform!(vis[:basics][:triad], Translation(0.0,0,0) ∘ LinearMap( Quat(q.s,q.v...) ))
  @show vee(convert(so3, convert(SO3, q)))
  sleep(0.05)
end



ans = Quaternion(0) * SO3(0) * so3(randn(3)) * TransformUtils.AngleAxis(pi/2, [0,0,1.0])

vee(convert(so3, ans))
