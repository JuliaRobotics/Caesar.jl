
#

using LinearAlgebra
using Gadfly
Gadfly.set_default_plot_size(35cm,20cm)


p1 = 0.7 + 0.7*im

b1 = 0.4 + 0.2*im
b2 = 0.3 + 0.9*im
b3 = 0.8 + 0.1*im

r1 = 100*norm(b1-p1)
r2 = 100*norm(b2-p1)
r3 = 100*norm(b3-p1)


N = 50000


# random range samples
Cc1 = (rand(Complex{Float64},N) .- b1)*100
Cc2 = (rand(Complex{Float64},N) .- b2)*100
Cc3 = (rand(Complex{Float64},N) .- b3)*100



A = zeros(100,100)

kr1 = 1 ./ (abs.(r1 .- norm.(Cc1)) )
kr2 = 1 ./ (abs.(r2 .- norm.(Cc2)) )
kr3 = 1 ./ (abs.(r3 .- norm.(Cc3)) )

kr1 .= exp.(kr1) .- 1
kr2 .= exp.(kr2) .- 1
kr3 .= exp.(kr3) .- 1

mask1 = rand(N) .< kr1
mask2 = rand(N) .< kr2
mask3 = rand(N) .< kr3


X1 = Cc1[mask1] .|> real .|> x->round(Int,x)
Y1 = Cc1[mask1] .|> imag .|> x->round(Int,x)
X1 .+= 100*real(b1)
Y1 .+= 100*imag(b1)
X1[X1 .== 0] .= 1
Y1[Y1 .== 0] .= 1
for i in 1:sum(mask1)
  A[X1[i],Y1[i]] += 1
end

X2 = Cc2[mask2] .|> real .|> x->round(Int,x)
Y2 = Cc2[mask2] .|> imag .|> x->round(Int,x)
X2 .+= 100*real(b2)
Y2 .+= 100*imag(b2)
X2[X2 .== 0] .= 1
Y2[Y2 .== 0] .= 1
for i in 1:sum(mask2)
  A[X2[i],Y2[i]] += 1
end

X3 = Cc3[mask3] .|> real .|> x->round(Int,x)
Y3 = Cc3[mask3] .|> imag .|> x->round(Int,x)
X3 .+= 100*real(b3)
Y3 .+= 100*imag(b3)
X3[X3 .== 0] .= 1
Y3[Y3 .== 0] .= 1
for i in 1:sum(mask3)
  A[X3[i],Y3[i]] += 1
end



pl = spy(A)



#