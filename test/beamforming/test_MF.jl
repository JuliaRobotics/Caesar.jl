
using Caesar
using Pkg
using DelimitedFiles

chirpFile = joinpath(Pkg.dir("Caesar"),"test","testdata","template.txt");
chirpIn = readdlm(chirpFile,',',Float64,'\n')

logfile = joinpath(Pkg.dir("Caesar"),"test","testdata","wavetest01.txt");
rawData = readdlm(logfile,',',Float64,'\n')  # waveform timeseries from hydrophone

nFFT = nextpow(2,size(rawData,1))
nPhones = 5

mf = Caesar.prepMF(chirpIn,nFFT,nPhones)
dataOut = zeros(Complex{Float64}, nFFT, nPhones)
mf(rawData,dataOut)

0

# using Plots
#
# plot(
#   hcat(
#     vcat(
#       rawData[:,1],
#       zeros(nFFT-size(rawData,1))
#     ),
#     real(dataOut[:,1])
#   )
# )




# What are we doing -- to do matched FILTERING
# 1.) A: take fft over all data
# 1b.) OPTIONAL:  A = A./(abs.(A).^p + ε),  p ∈ [0,1] -- normalize mag over all frequency, gives 1's but keeps phase -- PHAT transform
# 2.) B: take fft |> conj! of the replica
# 3.) element wise mutiply: C = A.*B
# 4.) ifft(C) gives correlation in time domain
