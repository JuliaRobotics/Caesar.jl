# use conventional beam forming

using Cairo, Fontconfig
using Gadfly
using JLD2, FileIO

using DSP
using Caesar

AMP.setForceEvalDirect!(true)

include(joinpath(dirname(@__FILE__),"ps3utils.jl"))


## Load the data
dataparent = joinpath(dirname(@__FILE__),"..","..","..","..","..","..","data","ps3")
datadir = joinpath(dataparent,"equilateral_22inches") #
# datadir = joinpath(dataparent,"demo")

# reload data

leftwavdata = loadPs3WavEnvelopes(joinpath(datadir,"left_wavdata_corrected.jld2"))
rightwavdata = loadPs3WavEnvelopes(joinpath(datadir,"right_wavdata_corrected.jld2"))


## Load the templates
templdir = joinpath(dataparent, "templateX200ms")
lefttemplate = loadPs3WavEnvelopes(joinpath(templdir, "template_left.jld2"), variable="templ", cutmax=false)
righttemplate = loadPs3WavEnvelopes(joinpath(templdir, "template_right.jld2"), variable="templ", cutmax=false)


leftwav = [leftwavdata[0] leftwavdata[1] leftwavdata[2] leftwavdata[3]]
rightwav = [rightwavdata[0] rightwavdata[1] rightwavdata[2] rightwavdata[3]]


## Spot check

Gadfly.plot(y=leftwav[:,1], Geom.line)
Gadfly.plot(y=rightwav[:,1], Geom.line)

## generate nonparametric bearing intensities from waveform data


intensityLeft = beamformBasic(leftwav, lefttemplate, soundSpeed=340)
intensityRight = beamformBasic(rightwav, righttemplate, soundSpeed=340)
# Gadfly.plot(x=azimuths, y=intensity, Geom.line)


pcLeft = intensityToCircularKDE(intensityLeft, SNRfloor=0.80, N=100)
pcRight = intensityToCircularKDE(intensityRight, SNRfloor=0.80, N=100)


pl = AMP.plotKDECircular([pcLeft; pcRight], scale=0.4, c=["blue";"green"])
pl.coord = Coord.Cartesian(aspect_ratio=1.0, xmin=-2, xmax=2, ymin=-2,ymax=2)
pl


##



pl |> PDF("/tmp/test.pdf",30cm,20cm)
pl |> SVG("/tmp/test.svg",10cm,10cm)

##

@async run(`evince /tmp/test.pdf`)


##

pl = AMP.plotKDECircular([pcLeft], scale=0.4, c=["red";"green"])

#
