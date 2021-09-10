module CommonUtils

using Caesar
using Images
using FileIO

export buildDEMSimulated, loadDEM!

"""
Loads a sample DEM (as if simulated) on a regular grid... Its the Grand Canyon, 18x18km, 17m
"""
function buildDEMSimulated( scale=1, N=100; 
                            x_is_north=true,
                            x_min::Real=-9000, x_max::Real=9000,
                            y_min::Real=-9000, y_max::Real=9000)
    #
    img_ = load(joinpath(dirname(dirname(pathof(Caesar))), "examples","dev","scalar","dem.png")) .|> Gray
    img_ = scale.*Float64.(img_)

    N_ = minimum([N; size(img_)...])
    img = img_[1:N_, 1:N_]
    
    # flip image so x-axis in plot is North and y-axis is West (ie img[-north,west], because top left is 0,0)
    _img_ = if x_is_north
        _img = collect(img')
        reverse(_img, dims=2)
    else
        img
    end

    x = range(x_min, x_max, length = size(_img_,1)) # North
    y = range(y_min, y_max, length = size(_img_,2)) # East

    return (x, y, _img_)
end

@deprecate getSampleDEM(w...;kw...) buildDEMSimulated(w...;kw...)





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





end