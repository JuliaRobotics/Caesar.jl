#=
Load a DEM as a Point3 set in a factor graph
=#

using Images
using FileIO
using Interpolations
using Caesar
using RoME
using RoME: MeanMaxPPE
using ProgressMeter

using TensorCast
using Gadfly
Gadfly.set_default_plot_size(35cm, 25cm)

##

prjPath = dirname(dirname(pathof(Caesar)))

# load dem (18x18km span, ~17m/px)
img = load(joinpath(prjPath, "examples","dev","scalar","dem.png")) .|> Gray
img = Float64.(img)
# crop to small region to improve performance
img = img[1:100, 1:100]

x = range(-9e3, 9e3, length = size(img)[1]) # North
y = range(-9e3, 9e3, length = size(img)[2]) # East

# point uncertainty - 2.5m horizontal, 1m vertical
# horizontal uncertainty chosen so that 3sigma is approx half the resolution
sigma = diagm([2.5, 2.5, 1.0])


function loadDEM!(  fg::AbstractDFG,
                    dem::Matrix, # assume grayscale image for now
                    x::AbstractVector,
                    y::AbstractVector;
                    solvable::Int=0,
                    marginalized::Bool=true,
                    meshEdgeSigma=diagm([1;1;1]),
                    refKey::Symbol = :simulated )
    """
    Load gridded elevation data `dem` into a factor graph `fg` as a collection
    of Point3 variables. Each variable is connected to its 4-neighborhood by
    relative Point3Point3 MvNormal constraints with mean defined by their
    relative position on the grid (`x`, `y`) and covariance `meshEdgeSigma`.
    """
    
    # assume regular grid
    dx, dy = x[2]-x[1], y[2]-y[1]
    for i in 1:length(x)
        for j in 1:length(y)
            s = Symbol("pt$(i)_$(j)") # unique identifier
            pt = addVariable!(fg, s, Point3, solvable=solvable)        
            setMarginalized!(pt, marginalized) # assume solveKey=:default
            
            # ...
            refVal = [x[i];y[j];dem[i,j]]
            simPPE = DFG.MeanMaxPPE(refKey, refVal, refVal, refVal)
            setPPE!(pt, refKey, typeof(simPPE), simPPE)
            
            # Regular grid triangulation:
            #  add factor to (i-1,j)     |
            #  add factor to (i, j-1)    -
            #  add factor to (i-1, j-1)  \

            # no edges to prev row on first row
            if i>1
                dVal1 = dem[i,j]-dem[i-1,j]
                f = Point3Point3(MvNormal([dx, 0, dVal1], meshEdgeSigma))
                addFactor!(fg, [Symbol("pt$(i-1)_$(j)"), s], f, solvable=solvable, graphinit=false)
            end
            
            # no edges to prev column on first column
            if j>1
                dVal2 = dem[i,j]-dem[i,j-1]
                f = Point3Point3(MvNormal([0, dy, dVal2], meshEdgeSigma))
                addFactor!(fg, [Symbol("pt$(i)_$(j-1)"),s], f, solvable=solvable, graphinit=false)
            end

            # no edges to add on first element
            if i>1 && j>1
                dVal3 = dem[i,j]-dem[i-1,j-1]
                f = Point3Point3(MvNormal([dx, dy, dVal3], meshEdgeSigma))
                addFactor!(fg,[Symbol("pt$(i-1)_$(j-1)"),s], f, solvable=solvable, graphinit=false)
            end
            
        end
    end

    nothing
end


## Testing 

# 0. init empty FG w/ datastore
fg = initfg()
storeDir = joinLogPath(fg,"data")
mkpath(storeDir)
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg, datastore)


# 1. load DEM
# 27.622065 seconds (6.64 M allocations: 25.847 GiB, 6.26% gc time)
@time loadDEM!(fg, img, (x), (y), meshEdgeSigma=sigma);

# 2. generate trajectory 
cb(g, lbl) = @show lbl
@time generateCanonicalFG_Helix2DSlew!(100, posesperturn=30, radius=500, dfg=fg, graphinit=false, slew_y=2/3, postpose_cb=cb)

# check: query variables
getPPE(fg, :x0, :simulated).suggested
getPPE(fg, :pt50_50, :simulated).suggested


# check: fetch and plot pose variables

traj = ls(fg, r"x\d+") |> sortDFG
traj_xy = traj .|> x->getPPE(fg, x, :simulated).suggested[1:2]

@cast _xy[i,j] := traj_xy[i][j]

plot(
    layer(x=_xy[:,1],y=_xy[:,2], Geom.point),
    layer(x=_xy[:,1],y=_xy[:,2], Geom.path)
)

# 3. simulate elevation measurements
dem = Interpolations.LinearInterpolation((x,y), img) # interpolated DEM
elevation(p) = dem[getPPE(fg, p, :simulated).suggested[1:2]'...]

poses = ls(fg, r"x\d+") |> sortDFG

# fetch interpolated elevation at ground truth position 

for p in poses
    e = elevation(p) 
    # add to variable

end

# 4a. add pairwise constraints between poses and points

# 4b. add pairwise constraints between poses (cross-over)



