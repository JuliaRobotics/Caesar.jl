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
# Launch the viewer application if it isn't running already:
# DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window();



setgeometry!(vis[:basics][:triad], Triad())
settransform!(vis[:basics][:triad], Translation(0.0,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/4,0,0,1.)  ))

setgeometry!(vis[:basics][:box], HyperRectangle(Vec(0.,0,0), Vec(1.,1,1)) )
settransform!(vis[:basics][:box], Translation(2.0,0,0) ∘ LinearMap(Rotations.AngleAxis(0.0,0,0,1.)  ))

# and a point cloud

pointcloud = PointCloud([[x, 0, 0.5] for x in linspace(-1, 1)])
pointcloud.channels[:rgb] = [RGB(g, (1-g), 0) for g in linspace(0, 1)]
setgeometry!(vis[:pointcloud], pointcloud)
delete!(vis[:pointcloud])
