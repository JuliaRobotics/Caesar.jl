
using Caesar, DelimitedFiles
using Gadfly, Cairo, Fontconfig
# using MAT
# using AbstractPlotting, Makie

using DocStringExtensions
include(joinpath(@__DIR__,"..","..","examples","marine","asv","kayaks","slamUtils.jl"))

fFloor = 250;
fCeil = 1750;
nFFT_czt = 1024;
fSampling = 37500.0
nPhones = 10;
soundSpeed = 1481;
azimuthDivs = 180;
azimuths = range(0,360,length=azimuthDivs)*pi/180;

# Load test data of length 10
# winstart = 0;
# posFile = joinpath(ENV["HOME"],"data","sas","test_array_positions.csv");
# arrayElemPos = readdlm(posFile,',',Float64,'\n')
# arrayElemPos = arrayElemPos[:,1:2]
# dataFile = joinpath(ENV["HOME"],"data","sas","test_array_waveforms.csv");
# rawWaveData = readdlm(dataFile,',',Float64,'\n')

#Load Experimental Data - 10 frames
# winstart = 160;
# rawWaveData = zeros(8000,nPhones);
# arrayElemPos = zeros(nPhones,2);
# for ele in winstart:winstart+nPhones-1
#     dataFile = joinpath(ENV["HOME"],"data", "sas", "06_20_sample","waveform$(ele).csv");
#     posFile = joinpath(ENV["HOME"],"data", "sas", "06_20_sample","nav$(ele).csv");
#     #dataFile = joinpath(ENV["HOME"],"liljondir", "kayaks", "20_gps_pos","waveform$(ele).csv");
#     #posFile = joinpath(ENV["HOME"],"liljondir", "kayaks", "20_gps_pos","nav$(ele).csv");
#     tempRead = readdlm(dataFile,',',Float64,'\n') #first element only
#     rawWaveData[:,ele-winstart+1] = adjoint(tempRead[1,:]);
#     tempRead = readdlm(posFile,',',Float64,'\n');
#     arrayElemPos[ele-winstart+1,:] = tempRead;
# end
# beacongt = [17.0499;1.7832];

# chirpFile = joinpath(ENV["HOME"],"data","sas","chirp250.txt");

# Synthetic data
winstart = 5;
posFile = joinpath(ENV["HOME"],"data","sas","syntheticArray.csv");
arrayElemPos = readdlm(posFile,',',Float64,'\n')
arrayElemPos = arrayElemPos[:,1:2]
dataFile = joinpath(ENV["HOME"],"data","sas","synthetic_data.csv");
rawWaveData = readdlm(dataFile,',',Float64,'\n')
chirpFile = joinpath(ENV["HOME"],"data","sas","chirp_signal_synth.csv");
nPhones = 11;
beacongt = [100,125];

chirpIn = readdlm(chirpFile,',',Float64,'\n')
FFTfreqs = collect(LinRange(fFloor,fCeil,nFFT_czt))

#Leave One Out Full CBF
leaveout = 6;
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
 Gadfly.layer(x=azimuths,y=real(dataOut), Geom.line),
 Gadfly.Coord.cartesian(xmin=0, xmax=2*pi),
 Guide.ylabel(nothing),Guide.xlabel("Azimuths (rad)")
 ) ; pl |> PDF("/tmp/test_CBF_LIE.pdf")

# LIE CBF - Assume Known Source, azi phase shift
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

#Try singlephaseshift
phaseshiftLOO = cztData[:,leaveout];
@time phaseShiftSingle!(lm, cfgLIE, azi, arrayElemPos[leaveout,1:2], phaseshiftLOO)

#Visualize residual at correct look angle
variation = -5:0.1:5;
allshifts = zeros(Complex{Float64},length(variation),length(variation),length(cztData[:,leaveout]));
dataOutRes = zeros(Complex{Float64}, nFFT_czt);

dxArr = arrayElemPos[1,1] - beacongt[1];
dyArr = arrayElemPos[1,2] - beacongt[2];
aziArr = atan(dyArr/dxArr)

cfgLIE = CBFFilterConfig(fFloor,fCeil,nFFT_czt,nPhones-1,[aziArr;],soundSpeed,FFTfreqs)
myCBFLIE = zeros(Complex{Float64}, getCBFFilter2Dsize(cfgLIE));
constructCBFFilter2D!(cfgLIE, arrayLIE, myCBFLIE, lm, dataTempHolder,delaysHolder)
cztDataHolder = zeros(Complex{Float64},size(cztDataLIE));
liebf!(dataOutRes, cztDataLIE, myCBFLIE, 1, cztDataHolder, normalize=true)

for xInd in 1:length(variation)
    for yInd in 1:length(variation)

        dx = arrayElemPos[leaveout,1] - beacongt[1] + variation[xInd];
        dy = arrayElemPos[leaveout,2] - beacongt[2] + variation[yInd];
        azi = atan(dy,dx);
        phaseshiftLOO = cztData[:,leaveout];
        phaseShiftSingle!(lm, cfgLIE, azi, arrayElemPos[leaveout,1:2], phaseshiftLOO)
        allshifts[xInd,yInd,:] = phaseshiftLOO .+ dataOutRes;
    end
end

# tmpfile = "/tmp/res$(winstart).mat";
# file = matopen(tmpfile, "w")
# write(file, "allshifts", allshifts)
# close(file)


#Visualize residual over many look angles
variation = range(-5,5,length=200);
uvec = arrayElemPos[leaveout,1:2]-beacongt
uvec ./= norm(uvec)
rvec = R(pi/2)*uvec;

allshiftsAziX = zeros(Complex{Float64},length(azimuths),length(variation),length(cztData[:,leaveout]));
dataOutRes = zeros(Complex{Float64}, nFFT_czt);
allshiftsAziY = zeros(Complex{Float64},length(azimuths),length(variation),length(cztData[:,leaveout]));
dataOutRes = zeros(Complex{Float64}, nFFT_czt);
allshiftsAzilin = zeros(Complex{Float64},length(azimuths),length(variation),length(cztData[:,leaveout]));
dataOutRes = zeros(Complex{Float64}, nFFT_czt);

zdx = zeros(length(azimuths),length(variation)); #plot vars
zdy = zeros(length(azimuths),length(variation));
zlin = zeros(length(azimuths),length(variation));

dataOutHolder = zeros(Complex{Float64}, nFFT_czt);

for aziInd in 1:length(azimuths)
    cfgLIE = CBFFilterConfig(fFloor,fCeil,nFFT_czt,nPhones-1,[azimuths[aziInd];],soundSpeed,FFTfreqs)
    myCBFLIE = zeros(Complex{Float64}, getCBFFilter2Dsize(cfgLIE));
    constructCBFFilter2D!(cfgLIE, arrayLIE, myCBFLIE, lm, dataTempHolder,delaysHolder)
    cztDataHolder = zeros(Complex{Float64},size(cztDataLIE));
    liebf!(dataOutRes, cztDataLIE, myCBFLIE, 1, cztDataHolder, normalize=true)

    for xInd in 1:length(variation)
            newPos = arrayElemPos[leaveout,1:2]+[variation[xInd];0];
            dx = newPos[1] - beacongt[1];
            dy = newPos[2] - beacongt[2];
            azi = atan(dy,dx);
            phaseshiftLOO = cztData[:,leaveout];
            phaseShiftSingle!(lm, cfgLIE, azi,newPos , phaseshiftLOO)
            zdx[aziInd,xInd] = 0.0;
            copy!(dataOutHolder,dataOutRes)
            @inbounds for i in 1:length(phaseshiftLOO)
              dataOutHolder[i] += phaseshiftLOO[i]
              zdx[aziInd,xInd] += norm(dataOutHolder[i])
            end
    end

    for yInd in 1:length(variation)
            # newPos = arrayElemPos[leaveout,1:2]+[0;variation[yInd]];
            dlin = variation[yInd]*rvec;
            newPos = arrayElemPos[leaveout,1:2]+dlin;

            dx = newPos[1] - beacongt[1];
            dy = newPos[2] - beacongt[2];
            azi = atan(dy,dx);
            phaseshiftLOO = cztData[:,leaveout];
            phaseShiftSingle!(lm, cfgLIE, azi,newPos, phaseshiftLOO)
            zdy[aziInd,yInd] = 0.0;
            copy!(dataOutHolder,dataOutRes)
            @inbounds for i in 1:length(phaseshiftLOO)
              dataOutHolder[i] += phaseshiftLOO[i]
              zdy[aziInd,yInd] += norm(dataOutHolder[i])
            end
    end

    for linInd in 1:length(variation)
            dlin = variation[linInd]*uvec;
            newPos = arrayElemPos[leaveout,1:2]+dlin;
            dx = newPos[1] - beacongt[1];
            dy = newPos[2] - beacongt[2];
            azi = atan(dy,dx);
            phaseshiftLOO = cztData[:,leaveout];
            phaseShiftSingle!(lm, cfgLIE, azi,newPos, phaseshiftLOO)
            zlin[aziInd,linInd] = 0.0;
            copy!(dataOutHolder,dataOutRes)
            @inbounds for i in 1:length(phaseshiftLOO)
              dataOutHolder[i] += phaseshiftLOO[i]
              zlin[aziInd,linInd] += norm(dataOutHolder[i])
          end
    end
end

# scene = AbstractPlotting.surface(azimuths, variation, z./maximum(z))
# Makie.save("plot.png", scene)
#
# tmpfile = "/tmp/resAzis$(winstart).mat";
# file = matopen(tmpfile, "w")
# write(file, "zdx", zdx)
# write(file, "zdy", zdy)
# write(file, "zlin", zlin)
# close(file)
