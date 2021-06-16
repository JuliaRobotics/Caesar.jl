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
        for j in 1:length(y)
            s = Symbol("pt$(i)_$(j)") 
            addVariable!(fg, s, Point3)
        
            f = PriorPoint3(MvNormal([x[i],y[i],img[i,j]],sigma))
            addFactor!(fg,[s], f, graphinit=false)
        end
    end

    # TODO smoothness constraints

    nothing
end


function getTM(dem::Matrix)
    # get z transition samples over x and y
    # NOTE: this works over a regular grid, so take care when using
    # with non-uniformly spaced points
    s = size(dem)
    dc = img[2:(s[1]-1), 2:(s[2]-1)]
    
    dx1 = dc - img[1:(s[1]-2), 2:(s[2]-1)]
    dx2 = dc - img[3:s[1],     2:(s[2]-1)]
    dy1 = dc - img[2:(s[1]-1), 1:(s[1]-2)]
    dy2 = dc - img[2:(s[1]-1), 3:s[2]]
    [[dx1[:];dx2[:]],[dy1[:];dy2[:]]]
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
