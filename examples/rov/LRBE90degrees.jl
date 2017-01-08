# visualize 90 degree yaw with ROV as bimodal example.

using Caesar
using Rotations
using CoordinateTransformations



# start visualizer with some scene
vc = startdefaultvisualization(draworigin=false)
sleep(3.0)
defaultscene01!(vc)

# load necessary data
rovt = loadmodel(:rov)
rovt(vc)


# ground truth positions
gt = Dict{Symbol, Vector{Float64}}()
gt[:x1] = zeros(6)
gt[:x2] = [4.0;4.0;0;0;0;-pi/2]
gt[:l1] = [4.0;0.0;0.7]

as = ArcPointsRangeSolve([0.0;0;0.0],[4;4;0.0]./sqrt(2),[-4.0;-4.0;0.0],4.0)
findaxiscenter!(as)
@show as.center, as.axis, as.angle

for t in linspace(0,1,100)
  x = -t*4
  y = t*4
  phi = pi/2.0*t
  am = Translation(x,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0)) ∘ LinearMap( Rotations.AngleAxis(phi,1.0,0,0))
  rovt(vc, am )
  sleep(0.05)
end

am = Translation(0,0,0) ∘ LinearMap(Rotations.AngleAxis(pi/2,0,0,1.0))
rovt(vc, am )
