using Caesar

x = 0.2:0.4:10
m = length(x)
f1 = 100; f2=50;
w = exp(-2im*pi*(f2-f1)/m/1000)

myczt = prepCZTFilter(m,w)
myczt.fftV
myczt.Awk2
out = myczt(collect(x))
