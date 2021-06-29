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

prjPath = dirname(dirname(pathof(Caesar)))
include( joinpath(prjPath, "examples","dev","scalar","CommonUtils.jl") )

function getLevelSet(data,x, y, level, tol=0.01)
    """
    Get the grid positions at the specified height (within the provided tolerance)
    """
    roi = data.-level
    mask = abs.(roi) .< tol

    idx2d = findall(mask)  # 2D indices
    pos = (v->[x[v.I[1]],y[v.I[2]]]).(idx2d)
    
    # recast to the appropriate shape
    @cast kp[i,j] := pos[j][i]
    collect(kp)
end

function fitKDE(x::AbstractArray, y::AbstractArray,
    support)
    # 1. set the bandwidth 
    x_spacing = x[2]-x[1]
    y_spacing = y[2]-y[1]
    kernel_bw = 0.7*0.5*(x_spacing + y_spacing) # 70% of the average spacing
    kernel_bw = [kernel_bw; kernel_bw]          # same bw in x and y
    # fit KDE
    P = kde!(support)
end

function sampleHeatmap(data, x, y, level, tol=0.01, N=10000)
    support = getLevelSet(img, x, y, level)
    P = fitKDE(x, y, support)
    sample(P, N)[1]
end

(x, y, dem) = CommonUtils.getSampleDEM()

pts = sampleHeatmap(dem, x, y, 0.5)

imscatter(img) = scatterplot(img[2,:], -img[1,:])
imdensity(img) = densityplot(img[2,:], -img[1,:])
imscatter(pts)
imdensity(pts)