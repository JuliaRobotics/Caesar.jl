
using ProprietaryFactors

fFloor = 250;
fCeil = 1750;
nFFT_czt = 1024;
fSampling = 37500.0
totalPhones = 10;
soundSpeed = 1481;
azimuthDivs = 180;
azimuths = linspace(0,360,azimuthDivs)*pi/180;

logfile = joinpath(Pkg.dir("ProprietaryFactors"),"test","testdata","syntheticArray.csv");
arrayElemPos = readdlm(logfile,',',Float64,'\n')
arrayElemPos = arrayElemPos[:,1:2]
logfile = joinpath(Pkg.dir("ProprietaryFactors"),"test","testdata","synthetic_data.csv");
rawWaveData = readdlm(logfile,',',Float64,'\n')

#chirpFile = joinpath(Pkg.dir("ProprietaryFactors"),"test","testdata","chirp_signal_synth.csv");
#chirpIn = readdlm(chirpFile,',',Float64,'\n')

# Perturb array positions
samples = 100
exPos =

cfg = CBFFilterConfig(arrayPosLIE,fFloor,fCeil,nFFT_czt,nPhones,azimuths,soundSpeed)
bfFilterLIE = zeros(Complex{Float64}, getCBFFilter2Dsize(cfg));
updateCBFFilter2D!(cfg, bfFilterLIE)
