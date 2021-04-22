# prototyping tools to help with boxy & friends

using Images, FileIO           # load elevation from PNG
using Interpolations
using Gadfly, Colors
using Optim
using Test

# some Gadfly stuff
Gadfly.set_default_plot_size(35cm,25cm)
st=style( major_label_font="CMU Serif",
          minor_label_font="CMU Serif",
          major_label_font_size=20pt,
          minor_label_font_size=20pt)


struct MeasurementSequence
    x::AbstractArray
    y::AbstractArray
end

function getOverlap(s1::MeasurementSequence, s2::MeasurementSequence)   
    (max(minimum(s1.x),minimum(s2.x)), min(maximum(s1.x),maximum(s2.x)))
end

function ssd(s1::MeasurementSequence, s2::MeasurementSequence;res::Float64=0.1)
    cr = getOverlap(s1, s2)
    x = cr[1]:res:cr[2]
    y1c = LinearInterpolation(s1.x, s1.y)
    y2c = LinearInterpolation(s2.x, s2.y)

    return (1.0/length(x))*sum(y1c(x)-y2c(x)).^2
end

function displace(s::MeasurementSequence, d::Float64)
    MeasurementSequence(s.x .+d, s.y)
end


img = load("/home/pvt/workspace/libraries/Caesar.jl/examples/dev/scalar/dem.png")
img = Float64.(img)
terrain = 1e3*Float64.(@view img[300,:])

## 0. parameters
n = 150
sigma_y = 1

## 1. test w/ globally-referenced sequences

s1 = MeasurementSequence(collect(LinRange(0,15,n)), terrain[1:n]+sigma_y*randn(n))
s2 = MeasurementSequence(collect(LinRange(5,20,n)), terrain[1+Int(n/3):1+Int(n/3)+n-1]+sigma_y*randn(n))

ssd(s1, s2) # should be low

p = plot(st)
push!(p, layer(x=s1.x, y=s1.y,Geom.line,color=[colorant"orange"]))
push!(p, layer(x=s2.x, y=s2.y,Geom.line,color=[colorant"hotpink"]))

f(x) = ssd(s1, displace(s2, x))
r = optimize(f, -10.0, 10.0)

@test abs(r.minimizer)<0.1

# show results
p = plot(st)
push!(p, layer(x=s1.x, y=s1.y,Geom.line,color=[colorant"orange"]))
s3 = displace(s2, r.minimizer)
push!(p, layer(x=s3.x, y=s3.y,Geom.line,color=[colorant"hotpink"]))

## 2. test w/ locally-referenced sequences

s1 = MeasurementSequence(collect(LinRange(0,15,n)), terrain[1:n]+sigma_y*randn(n))
s2 = MeasurementSequence(collect(LinRange(0,15,n)), terrain[1+Int(n/3):1+Int(n/3)+n-1]+sigma_y*randn(n))

ssd(s1, s2) # should be low

p = plot(st)
push!(p, layer(x=s1.x, y=s1.y,Geom.line,color=[colorant"orange"]))
push!(p, layer(x=s2.x, y=s2.y,Geom.line,color=[colorant"hotpink"]))

f(x) = ssd(s1, displace(s2, x))
r = optimize(f, -10.0, 10.0)

@test abs(r.minimizer-5.0)<0.1

# show results
p = plot(st)
push!(p, layer(x=s1.x, y=s1.y,Geom.line,color=[colorant"orange"]))
s3 = displace(s2, r.minimizer)
push!(p, layer(x=s3.x, y=s3.y,Geom.line,color=[colorant"hotpink"]))


## 3. slightly more practical use - map ssd to probability

s1 = MeasurementSequence(collect(LinRange(0,15,n)), terrain[1:n]+sigma_y*randn(n))
s2 = MeasurementSequence(collect(LinRange(0,15,n)), terrain[1+Int(n/3):1+Int(n/3)+n-1]+sigma_y*randn(n))

p = plot(st)
push!(p, layer(x=s1.x, y=s1.y, Geom.line,color=[colorant"orange"]))
push!(p, layer(x=s2.x, y=s2.y, Geom.line,color=[colorant"hotpink"]))

f(x) = ssd(s1, displace(s2, x))
qr = collect(LinRange(-10.0,10.0,10001))
v = f.(qr)

k = abs(maximum(v))
pv = v./k
pv = exp.(-pv)
pv ./= sum(pv)
plot(x=qr,y=pv,Geom.line,Scale.y_log)
qr[argmax(pv)]