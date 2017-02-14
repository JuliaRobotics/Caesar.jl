# test DrakeVisualizer

using Caesar, RoME, TransformUtils


vc = startdefaultvisualization()


using DrakeVisualizer

using GeometryTypes

using CoordinateTransformations

import DrakeVisualizer: Triad

DrakeVisualizer.new_window();

tr = Triad()

viz = Visualizer()

@show typeof(viz)

box = HyperRectangle(Vec(0.,0,0), Vec(1.,1,1))

setgeometry!(viz[:origin], tr)

settransform!(viz[:origin], Translation(0.0,0,0) ∘ LinearMap(CoordinateTransformations.AngleAxis(0.0,0,0,1.)  ))



model = Visualizer(box)
