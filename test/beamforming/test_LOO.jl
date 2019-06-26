
using Caesar, DelimitedFiles
using Gadfly, Cairo, Fontconfig

fFloor = 250;
fCeil = 1750;
nFFT_czt = 1024;
fSampling = 37500.0
nPhones = 10;
soundSpeed = 1481;
azimuthDivs = 180;
azimuths = range(0,360,length=azimuthDivs)*pi/180;

# Load test data of length 10
# posFile = joinpath(ENV["HOME"],"data","sas","test_array_positions.csv");
# arrayElemPos = readdlm(posFile,',',Float64,'\n')
# arrayElemPos = arrayElemPos[:,1:2]
# dataFile = joinpath(ENV["HOME"],"data","sas","test_array_waveforms.csv");
# rawWaveData = readdlm(dataFile,',',Float64,'\n')

#Load Experimental Data - 10 frames
winstart = 1600;
rawWaveData = zeros(8000,nPhones);
arrayElemPos = zeros(nPhones,2);
for ele in winstart:winstart+nPhones-1
    dataFile = joinpath(ENV["HOME"],"data", "sas", "sample_data","waveform$(ele).csv");
    posFile = joinpath(ENV["HOME"],"data", "sas", "sample_data","nav$(ele).csv");
    tempRead = readdlm(dataFile,',',Float64,'\n') #first element only
    rawWaveData[:,ele-winstart+1] = adjoint(tempRead[1,:]);
    tempRead = readdlm(posFile,',',Float64,'\n');
    arrayElemPos[ele-winstart+1,:] = tempRead;
end

chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");

FFTfreqs = collect(LinRange(fFloor,fCeil,nFFT_czt))

#Leave One Out Full CBF
leaveout = 3;
elementset = setdiff(Int[1:nPhones;],[leaveout;]);
arrayLIE = arrayElemPos[elementset,:];

cfg = CBFFilterConfig(fFloor,fCeil,nFFT_czt,nPhones-1,azimuths,soundSpeed,FFTfreqs)
myCBF = zeros(Complex{Float64}, getCBFFilter2Dsize(cfg));
lm = zeros(2);
dataTempHolder = zeros(Complex{Float64},nPhones-1,nFFT_czt)
delaysHolder = FFTfreqs;
@time constructCBFFilter2D!(cfg, arrayLIE, myCBF, lm, dataTempHolder,delaysHolder)

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

#Try CBF over LIE
dataOut = zeros(length(azimuths));
cztDataLIE = cztData[:,elementset];
temp1 = zeros(Complex{Float64},nFFT_czt);
temp2 = zeros(Complex{Float64},size(cztDataLIE));
@time CBF2D_DelaySum!(cfg, cztDataLIE, dataOut,temp1,temp2,myCBF)

pl = Gadfly.plot(
 Gadfly.layer(y=real(dataOut), Geom.line),
 #Gadfly.Coord.cartesian(xmin=0, xmax=m),
 Guide.ylabel(nothing),Guide.xlabel("Samples")
 ) ; pl |> PDF("/tmp/test_CBF_LIE.pdf")

# LIE CBF - Assume Known Source, azi phase shift
beacongt = [17.0499;1.7832];
dataOutLIE = zeros(Complex{Float64}, 1);
dx = arrayLIE[1,1] - beacongt[1];
dy = arrayLIE[1,2] - beacongt[2];
azi = atan(dy,dx);
cfgLIE = CBFFilterConfig(fFloor,fCeil,nFFT_czt,nPhones-1,[azi;],soundSpeed,FFTfreqs)
myCBFLIE = zeros(Complex{Float64}, getCBFFilter2Dsize(cfgLIE));
@time constructCBFFilter2D!(cfgLIE, arrayLIE, myCBFLIE, lm, dataTempHolder,delaysHolder)

#Try Correlation
cztDataHolder = zeros(Complex{Float64},size(cztDataLIE));
@time liebf!(dataOutLIE, cztDataLIE, myCBFLIE, 1, cztDataHolder, normalize=true)

phaseshiftLOO = cztData[:,leaveout];
@time phaseShiftSingle!(lm, cfgLIE, azi, arrayElemPos[leaveout,1:2], phaseshiftLOO)

savethis = phaseshiftLOO;
