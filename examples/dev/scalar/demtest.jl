#=
Load a DEM as a Point3 set in a factor graph
=#

using Images
using FileIO
using Caesar
using RoME


prjPath = dirname(dirname(pathof(Caesar)))

# load dem (18x18km span, ~17m/px)
img = load(joinpath(prjPath, "examples","dev","scalar","dem.png")) .|> Gray
img = Float64.(img)

x = range(-9e3, 9e3, length = size(img)[1]) # North
y = range(-9e3, 9e3, length = size(img)[2]) # East

# point uncertainty - 2.5m horizontal, 1m vertical
# horizontal uncertainty chosen so that 3sigma is approx half the resolution
sigma = [2.5, 2.5, 1.0]

function loadDEM!(  fg::AbstractDFG,
                    dem::Matrix,
                    x::Vector,
                    y::Vector)

    for i in 1:length(x)
        for j in length(y)
            s = Symbol("pt$(i)_$(j)") 
            addVariable!(fg, s, Point3)
        
            f = PriorPoint3(MvNormal([x[i],y[i],img[i,j]],sigma))
            addFactor!(fg,[s], f, graphinit=false)
        end
    end
    nothing
end

fg = initfg()

loadDEM!(fg, img, Vector(x), Vector(y))
#=
ERROR: MethodError: no method matching getSample(::WARNING: both Manifolds and TransformUtils export "vee"; uses of it in module ApproxManifoldProducts must be qualified
WARNING: both Manifolds and TransformUtils export "vee!"; uses of it in module ApproxManifoldProducts must be qualified
CalcFactor{WARNING: both Rotations and ApproxManifoldProducts export "AngleAxis"; uses of it in module RoME must be qualified
WARNING: both ApproxManifoldProducts and TransformUtils export "rotate!"; uses of it in module RoME must be qualified
WARNING: both ManifoldsBase and ApproxManifoldProducts export "vee"; uses of it in module RoME must be qualified
WARNING: both ManifoldsBase and ApproxManifoldProducts export "vee!"; uses of it in module RoME must be qualified
=#
