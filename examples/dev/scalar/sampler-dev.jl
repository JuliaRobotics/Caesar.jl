using Images
using ImageView
using FileIO
using Interpolations
using Caesar
using RoME
using RoME: MeanMaxPPE
using ProgressMeter
using ImageView
using Statistics

using TensorCast
using UnicodePlots
# using Gadfly
# Gadfly.set_default_plot_size(35cm, 25cm)

using TensorCast

##

prjPath = dirname(dirname(pathof(Caesar)))
include( joinpath(prjPath, "examples","dev","scalar","CommonUtils.jl") )
using .CommonUtils


##

x, y, dem = buildDEMSimulated(1, 100, x_is_north=true) 

imshow(dem)


##

# z_elevation = 0.5,      # actual measurement
# sigma_elevation = 0.001 # measurement uncertainty
hmd = HeatmapDensityRegular(dem, (x,y), 0.5, 0.001)

# pr = Mixture(PriorPoint2, [hmd; MvNormal([0;0], [....])], [0.5;0.5])
# addFactor!(fg, [:x17;], pr)


##

M = TranslationGroup(2)
pts_ = [sampleTangent(M, hmd) for _ in 1:10000]


# pseudo code
# getSample(hmd, 100, hint_belief)


##

@cast pts[i,j] := pts_[j][i]

imscatter(img) = scatterplot(img[2,:], -img[1,:])
imdensity(img) = densityplot(img[2,:], -img[1,:])
imscatter(pts)
imdensity(pts)