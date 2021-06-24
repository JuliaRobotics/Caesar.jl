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
cb(g, lbl) = @show lbl # modify to generate elevation measurements (data/smallData as in Boxy) and priors
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




## ## DEV TESTING BELOW

using ImageView
using UnicodePlots

imshow(img)

## Aliasing sampler on sparse selection of points

@info "DEM value range" minimum(img) maximum(img)



function getLevelSet(data,x, y, level, tol=0.01)
    """
    Get the grid positions at the specified height
    """
    roi = data.-level
    mask = abs.(roi) .< tol

    idx2d = findall(mask)  # 2D indices
    # idx   = i:length(idx)
    (v->[x[v.I[1]],y[v.I[2]]]).(idx2d)
end

# go bigger than measurement noise, to also accomodate heatmap sampling process errors
R_meas = 1e-4

# selection of interest, assume elevation measurement aroun d0.5 is of interest
level = 0.5
img_soi = img .- level
immask = abs.(img_soi) .< 0.01

imshow(immask)

img_sparse = img_soi[immask]
idx = findall(immask)

idx_pos = (v->[x[v[1]],y[v[2]]]).(idx)

x_spacing = x[2]-x[1]
y_spacing = y[2]-y[1]

# imgT = zeros(100,100)
# imgT[10,:] .= 1.0

# imshow(imgT)

# OPTION A
kerBw_ = 0.7*0.5*(x_spacing + y_spacing) # 70%
kerBw = [kerBw_; kerBw_]
@cast kerPos_[i,j] := idx_pos[j][i]
kerPos = collect(kerPos_)
P_pre = kde!(kerPos, kerBw)
pts_preIS = sample(P_pre, 10000)[1]

# draw flipped so consistent with imshow img[i,j] == img[x,y]
imscatter(img_) = scatterplot(img_[2,:], -img_[1,:])
imscatter(pts_preIS)

#
densityplot(pts_preIS[2,:], -pts_preIS[1,:])

# weigth the samples according to interpolated heatmap



# still need heatmap interpolation eval for weighting
# weight = heatmap(pts_preIS)
# R_meas helps control sample process sensitivity
hm = Interpolations.LinearInterpolation((x,y), img .- level ) # interpolated DEM

@cast vec_preIS[j][i] := pts_preIS[i,j]

delev = Vector{Float64}(undef, length(vec_preIS))

function hack(w_, i, u)
    w_[i] = hm(u...)
    nothing
end

##

using ProgressMeter

##

@showprogress 1 "wait what...?" for (i,u) in enumerate(vec_preIS)
  @show i, u
  if 9000 < abs(u[1]) || 9000 < abs(u[2]) 
    delev[i] = 0.0
    continue
  end
  hack(delev, i, u)
end

##

weights = exp.(-(0.5/R_meas)*(abs.(delev)).^2)
weights ./= sum(weights)

histogram(weights, nbins=20, closed=:left)

# get final kde

# bw = [0.1;0.1]
prior = kde!(pts_preIS, kerBw)



##


# # OPTION B
# # where are the values of interest
# # make a distribution
# img_sparse .^= 2
# img_pdf = exp.(-img_sparse*0.5/R_meas)
# img_pdf ./= sum(img_pdf)
# bss = AliasingScalarSampler([id_idx;]./1.0, img_pdf)
# id_smpls = rand(bss, 10000) .|> Int



# ## how to super sample massive low res grid 2D scalar field

# # binary threshold the gridded heatmap

# # OPTION A
# # P_pre is weighted KDE kernels with fat bandwidth at each grid point (scalar dictates weight of kernel)
# # pts_preIS = sample(P_pre, 10000)
# # weight = heatmap(pts_preIS)

# # OPTION B
# # get first "grid points of interest" from heatmap with aliasing sampler 
# # mush those grid points to loose clusters (conv with Guassian)

# # importance weight those loose cluster points against the heatmap-interpolated
# N = 1000
# ## how to build weighted KDE (not full DEM pipeline yet)
# # mock heatmap
# mock = MvNormal([0.0;0.0], [1.0 0.5; 0.5 1.0])
# pts = rand(Mv, N)
# # mock mask
# mask = pts[2,:] .< 0.5

# # importance weighted samples from "heatmap
# weights = ones(N)
# weights[mask] .= 0.01
# weights ./= sum(weights)
# postMushWeightedPts = pts

# bw = [0.1;0.1]
# # this is the KDE belief to build a prior for localizaton from low res map (final step)
# P = kde!(postMushWeightedPts, bw, weights)






##




##

using KernelDensityEstimatePlotting
Gadfly.set_default_plot_size(35cm,25cm)

plotKDE(prior)

#