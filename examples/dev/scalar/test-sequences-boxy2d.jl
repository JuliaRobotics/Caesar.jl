"""
prototyping tools to help with boxy & friends
"""

# using Images, FileIO           # load elevation from PNG
using FileIO
using Interpolations
using Gadfly, Colors
using Optim
using Test
using JSON2
import Caesar

prjPath = pkgdir(Caesar)
include( joinpath(prjPath, "examples","dev","scalar","Sequences.jl") )

# some Gadfly stuff
# Gadfly.set_default_plot_size(35cm,25cm)
# st=style( major_label_font="CMU Serif",
#           minor_label_font="CMU Serif",
#           major_label_font_size=20pt,
#           minor_label_font_size=20pt)


## Load unique test data

datapath = joinpath(prjPath, "examples", "dev", "scalar", "data")


fid = open(joinpath(datapath, "seq_data_a.json"), "r")
myData_a = JSON2.read(fid, Dict{Symbol, Vector{Float64}}); close(fid)
fid = open(joinpath(datapath, "seq_data_b.json"), "r")
myData_b = JSON2.read(fid, Dict{Symbol, Vector{Float64}}); close(fid)

xseq_a = myData_a[:x_seq]
zseq_a = myData_a[:z_seq]

xseq_b = myData_b[:x_seq]
zseq_b = myData_b[:z_seq]

s_a  = Sequences.MeasurementSequence(myData_a[:x_seq], myData_a[:z_seq])
s_b  = Sequences.MeasurementSequence(myData_b[:x_seq], myData_b[:z_seq])



## Test 1. Globally-referenced sequences on correct grid (zero displacement)

@test Sequences.ssd(s_a,s_b) < 1e-3 #FIXME replace with test value

# find relative position through minimizer 
f(x) = Sequences.ssd(s_a, Sequences.displace(s_b, x))
r = Optim.optimize(f, -10.0, 10.0)
@test abs(r.minimizer) < 0.1 # valid for low noise



## Test 2. Locally-referenced
s_a  = Sequences.MeasurementSequence(myData_a[:x_seq], myData_a[:z_seq])
s_b  = Sequences.MeasurementSequence(myData_a[:x_seq], myData_b[:z_seq])
f(x) = Sequences.ssd(s_a, Sequences.displace(s_b, x))
r = Optim.optimize(f, -10.0, 10.0)
@test r.minimizer - 5  < 0.1 # valid for low noise


Gadfly.set_default_plot_size(35cm, 25cm) # make smaller for bigger font
st=style( major_label_font="CMU Serif",
          minor_label_font="CMU Serif",
          major_label_font_size=20pt,
          minor_label_font_size=20pt)
# plot input
p = plot(st)
push!(p, layer(x=s_a.x, y=s_a.y, Geom.line,color=[colorant"orange"]))
push!(p, layer(x=s_b.x, y=s_b.y, Geom.line,color=[colorant"hotpink"]))

# plot output/aligned sequences
p = plot(st)
push!(p, layer(x=s_a.x, y=s_a.y,Geom.line,color=[colorant"orange"]))
s3 = Sequences.displace(s_b, r.minimizer)
push!(p, layer(x=s3.x, y=s3.y,Geom.line,color=[colorant"hotpink"]))

# plot intensity (correlator output)
qr = collect(LinRange(-10.0,10.0,1001))  # displacement query range
v = f.(qr)
k = abs(maximum(v))
pv = v./k
pv = exp.(-pv)
pv ./= sum(pv)
plot(x=qr,y=pv,Geom.line) #,Scale.y_log)
qr[argmax(pv)] # most likely displacement





Sequences.ssd(s1, s2) # should be low

p = plot(st)
push!(p, layer(x=s1.x, y=s1.y,Geom.line,color=[colorant"orange"]))
push!(p, layer(x=s2.x, y=s2.y,Geom.line,color=[colorant"hotpink"]))

f(x) = Sequences.ssd(s1, Sequences.displace(s2, x))
r = optimize(f, -10.0, 10.0)

@test abs(r.minimizer - 5.0)<0.1

# show results
p = plot(st)
push!(p, layer(x=s1.x, y=s1.y,Geom.line,color=[colorant"orange"]))
s3 = Sequences.displace(s2, r.minimizer)
push!(p, layer(x=s3.x, y=s3.y,Geom.line,color=[colorant"hotpink"]))


## 3. slightly more practical use - map ssd to probability

s1 = Sequences.MeasurementSequence(collect(LinRange(0,15,n)), terrain[1:n]+sigma_y*randn(n))
s2 = Sequences.MeasurementSequence(collect(LinRange(0,15,n)), terrain[1+Int(n/3):1+Int(n/3)+n-1]+sigma_y*randn(n))

p = plot(st)
push!(p, layer(x=s1.x, y=s1.y, Geom.line,color=[colorant"orange"]))
push!(p, layer(x=s2.x, y=s2.y, Geom.line,color=[colorant"hotpink"]))

f(x) = Sequences.ssd(s1, Sequences.displace(s2, x))
qr = collect(LinRange(-10.0,10.0,10001))
v = f.(qr)

k = abs(maximum(v))
pv = v./k
pv = exp.(-pv)
pv ./= sum(pv)
plot(x=qr,y=pv,Geom.line,Scale.y_log)
qr[argmax(pv)]