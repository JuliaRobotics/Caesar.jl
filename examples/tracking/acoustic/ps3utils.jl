# beamform 2d util

function lastindmax(x)
   k = 1
   m = x[1]
   @inbounds for i in eachindex(x)
       if x[i]>=m
           k = i
           m = x[i]
       end
   end
   return k
end

function findMaxEnvelope(timeseries::Vector{Float64};
                         windowlen::Int=2^16,
                         fs::Int=192000,
                         tukeyalpha::Float64=0.7  )
  #
  window = tukey(windowlen, tukeyalpha)
  convres = conv(abs.(timeseries), window)
  stw = lastindmax(convres) - 2^16
  spw = stw + 2^16-1
  return stw, spw
end

function loadPs3WavEnvelopes(wavfilepath::String; variable="wavdata", cutmax::Bool=true)
  wavdata = load(wavfilepath, variable)
  if cutmax
    stw, spw = findMaxEnvelope(wavdata[0])
    for (k,va) in wavdata
      wavdata[k] = wavdata[k][stw:spw]
    end
  end
  return wavdata
end




function beamformBasic(csvWaveData::Array{Float64,2}, chirpIn; soundSpeed=330.0)::Vector{Float64}

  fFloor = 3950;
  fCeil = 8050;
  fSampling = 192000;
  nFFT_full = 2^16;
  nFFT_czt = 2^14;
  azimuthDivs = 180;
  # rngSNRfloor = cfgd["range_snr_floor"];
  # aziSNRfloor = cfgd["azi_snr_floor"];

  #Params subject to change/ Heuristics
  totalPhones = 4
  azimuths = LinRange(0,360,azimuthDivs)*pi/180;

  #Data preprocessing MF + CZT
  w = exp(-2im*pi*(fCeil-fFloor)/(nFFT_czt*fSampling))
  a = exp(2im*pi*fFloor/fSampling)

  arrayPos = collect([0.0 0.0 0.0 0.0; 0.03 0.01 -0.01 -0.03]')

  # Matched Filter on Data In
  nFFT_full = nextpow(2,size(csvWaveData,1))  # MF
  mfData = zeros(Complex{Float64}, nFFT_full, totalPhones)
  mf = prepMF(chirpIn, nFFT_full,totalPhones) # MF
  mf(csvWaveData, mfData) # MF
  cztData = zeros(Complex{Float64}, nFFT_czt, totalPhones)
  filterCZT = prepCZTFilter(nFFT_full, totalPhones, w, nFFT_czt,a)
  filterCZT(mfData, cztData)

  # CBF Filter shared by
  FFTfreqs = collect(LinRange(fFloor,fCeil,nFFT_czt))
  cfgTotal = CBFFilterConfig(fFloor,fCeil,nFFT_czt,totalPhones,azimuths,soundSpeed, FFTfreqs)

  sourceXY = zeros(2)
  BFTemp = zeros(Complex{Float64}, totalPhones, nFFT_czt)
  CBFFull = zeros(Complex{Float64}, getCBFFilter2Dsize(cfgTotal))
  BFOutFull = zeros(length(azimuths))

  constructCBFFilter2D!(cfgTotal, arrayPos, CBFFull, sourceXY, BFTemp)

  temp1 = zeros(Complex{Float64}, nFFT_czt)
  temp2 = zeros(Complex{Float64}, nFFT_czt, totalPhones)

  CBF2D_DelaySum!(cfgTotal, cztData, BFOutFull, temp1, temp2, CBFFull)

  intensity = norm.(BFOutFull)
  intensity ./= sum(intensity)

  return intensity
end


function intensityToCircularKDE(intensity::Vector{Float64}; N=100, SNRfloor::Float64=0.8)
  # increase resolution for aliasing sampler
  azi_weights = (repeat(norm.(intensity), 1,8)')[:]  # BFOutFull
  azi_weights ./= sum(azi_weights)

  azi_domain = collect(LinRange(0,2pi-1e-10,length(azi_weights)))

  bss = AliasingScalarSampler(azi_domain, azi_weights, SNRfloor=SNRfloor)

  ptsC = TransformUtils.wrapRad.(rand(bss,N))

  return manikde!(reshape(ptsC,1,:), (:Circular,))
end



#
