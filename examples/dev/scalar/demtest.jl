using RoME: MeanMaxPPE
#=
Load a DEM as a Point3 set in a factor graph
=#

using Images
using FileIO
using Caesar
using RoME
using ProgressMeter

using TensorCast
using Gadfly
Gadfly.set_default_plot_size(35cm, 25cm)



##

prjPath = dirname(dirname(pathof(Caesar)))

# load dem (18x18km span, ~17m/px)
img = load(joinpath(prjPath, "examples","dev","scalar","dem.png")) .|> Gray
img = Float64.(img)
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
    #
    # assume regular grid
    dx, dy = x[2]-x[1], y[2]-y[1]
    @showprogress 1 "loading rows..." for i in 1:length(x)
        for j in 1:length(y)
            s = Symbol("pt$(i)_$(j)") # unique identifier
            v_ij = addVariable!(fg, s, Point3, solvable=solvable)        
            setMarginalized!(v_ij, marginalized) # assume solveKey=:default
            refVal = [x[i];y[j];dem[i,j]]
            simPPE = DFG.MeanMaxPPE(refKey, refVal, refVal, refVal)
            setPPE!(v_ij, refKey, typeof(simPPE), simPPE)
            
            # Regular grid triangulation
            # add factor to (i-1,j) 
            # add factor to (i, j-1)
            # add factor to (i-1, j-1)

            # skip first corner
            # i == 1 && j == 1 ? continue : @warn("skip corner")
            # eg i=2, j=1
            # @show i,j
            if i>1
                dVal1 = dem[i,j]-dem[i-1,j]
                f = Point3Point3(MvNormal([dx, 0, dVal1], meshEdgeSigma))
                addFactor!(fg, [Symbol("pt$(i-1)_$(j)"), s], f, solvable=solvable, graphinit=false )
            end
            
            # skip out on first column
            # j == 1 ? continue : nothing
            if j>1
                dVal2 = dem[i,j]-dem[i,j-1]
                f = Point3Point3(MvNormal([0, dy, dVal2], meshEdgeSigma))
                addFactor!(fg, [Symbol("pt$(i)_$(j-1)"),s], f, solvable=solvable, graphinit=false )
            end

            # skip out on first row
            # i == 1 ? continue : nothing

            if i>1 && j>1
                dVal3 = dem[i,j]-dem[i-1,j-1]
                f = Point3Point3(MvNormal([dx, dy, dVal3], meshEdgeSigma))
                addFactor!(fg,[Symbol("pt$(i-1)_$(j-1)"),s], f, solvable=solvable, graphinit=false )
            end
            
        end
    end


    nothing
end


##


# function getTM(dem::Matrix)
#     # get z transition samples over x and y
#     # NOTE: this works over a regular grid, so take care when using
#     # with non-uniformly spaced points
#     s = size(dem)
#     dc = img[2:(s[1]-1), 2:(s[2]-1)]
    
#     dx1 = dc - img[1:(s[1]-2), 2:(s[2]-1)]
#     dx2 = dc - img[3:s[1],     2:(s[2]-1)]
#     dy1 = dc - img[2:(s[1]-1), 1:(s[1]-2)]
#     dy2 = dc - img[2:(s[1]-1), 3:s[2]]
#     [[dx1[:];dx2[:]],[dy1[:];dy2[:]]]
# end

fg = initfg()

##

loadDEM!(fg, img, (x), (y));


##

#callback
cb(g, lbl) = @show lbl


generateCanonicalFG_Helix2DSlew!(100, posesperturn=30, radius=500, dfg=fg, graphinit=false, slew_y=2/3, postpose_cb=cb)


##


getPPE(fg, :x0, :simulated).suggested

getPPE(fg, :pt50_50, :simulated).suggested


## fetch and plot pose variables

traj = ls(fg, r"x\d+") |> sortDFG

traj_xy = traj .|> x->getPPE(fg, x, :simulated).suggested[1:2]


@cast _xy[i,j] := traj_xy[i][j]

plot(
    layer(x=_xy[:,1],y=_xy[:,2], Geom.point),
    layer(x=_xy[:,1],y=_xy[:,2], Geom.path)
)

##


