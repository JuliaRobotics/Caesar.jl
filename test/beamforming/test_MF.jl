
using Caesar
using Pkg
using DelimitedFiles

chirpFile = joinpath(ENV["HOME"],"data", "sas","chirp250.txt");
chirpIn = readdlm(chirpFile,',',Float64,'\n')

logFile = joinpath(ENV["HOME"],"liljondir", "kayaks","20_gps_pos","waveform300.csv");
rawData = readdlm(logFile,',',Float64,'\n')  # waveform timeseries from hydrophone

#tranpose data to get format datavec x nphones
rawDataAdj = zeros(Float64,size(rawData,2),size(rawData,1))
adjoint!(rawDataAdj,rawData) #data x phones

nFFT = nextpow(2,size(rawDataAdj,1))
nPhones = 5

mf = Caesar.prepMF(chirpIn,nFFT,nPhones)
dataOut = zeros(Complex{Float64}, nFFT, nPhones)
@time mf(rawDataAdj,dataOut)

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
