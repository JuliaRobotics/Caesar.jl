
using Caesar, DelimitedFiles
using Gadfly

fFloor = 250;
fCeil = 1750;
nFFT_czt = 1024;
fSampling = 37500.0
nPhones = 10;
soundSpeed = 1481;
azimuthDivs = 180;
azimuths = range(0,360,length=azimuthDivs)*pi/180;

posFile = joinpath(ENV["HOME"],"data","sas","test_array_positions.csv");
arrayElemPos = readdlm(posFile,',',Float64,'\n')
arrayElemPos = arrayElemPos[:,1:2]
dataFile = joinpath(ENV["HOME"],"data","sas","test_array_waveforms.csv");
rawWaveData = readdlm(dataFile,',',Float64,'\n')
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");

FFTfreqs = collect(LinRange(fFloor,fCeil,nFFT_czt))

cfg = CBFFilterConfig(fFloor,fCeil,nFFT_czt,nPhones,azimuths,soundSpeed,FFTfreqs)
myCBF = zeros(Complex{Float64}, getCBFFilter2Dsize(cfg));
lm = zeros(2,1);
dataTempHolder = zeros(Complex{Float64},nPhones,nFFT_czt)
@time constructCBFFilter2D!(cfg, arrayElemPos, myCBF, lm, dataTempHolder)

# MF and CZT
w = exp(-2im*pi*(fCeil-fFloor)/(nFFT_czt*fSampling))
a = exp(2im*pi*fFloor/fSampling)

chirpIn = readdlm(chirpFile,',',Float64,'\n')

#Matched Filter on Data In
nFFT_full = nextpow(2,size(rawWaveData,1))  # MF
mfData = zeros(Complex{Float64}, nFFT_full, nPhones)
mf = prepMF(chirpIn,nFFT_full,nPhones) # MF
mf(rawWaveData,mfData) # MF

cztData = zeros(Complex{Float64}, nFFT_czt,nPhones)
filterCZT = prepCZTFilter(nFFT_full,nPhones,w,nFFT_czt,a)
filterCZT(mfData,cztData)

# CBF step
dataOut = zeros(length(azimuths));
temp1 = zeros(Complex{Float64},nFFT_czt);
temp2 = zeros(Complex{Float64},size(cztData));
@time CBF2D_DelaySum!(cfg, cztData, dataOut,temp1,temp2,myCBF)
