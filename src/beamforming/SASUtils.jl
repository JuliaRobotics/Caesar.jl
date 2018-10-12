

"""
    $(SIGNATURES)

Convenience function to load yaml config files, commonly used to prepare the required beamforming computational objects.
"""
function loadConfigFile(file::A) where {A <: AbstractString}
  YAML.load(open(file))
end


"""
    $(SIGNATURES)

Prepare a sas2d factor to use in the factor graph where `totalPhones` is the size of the SAS array.  Pass a known `cfgd::Dict{String,} for faster load times.`
"""
function prepareSAS2DFactor(totalPhones::Int,
                            csvWaveData::Array{<:Real};
                            cfgd::Dict=Dict(),
                            rangemodel=:Rayleigh )
  #
  # @assert size(csvWaveData,2) == totalPhones
  #Constant Acoustic Params
  fFloor = cfgd["freq_lower_cutoff"];
  fCeil = cfgd["freq_upper_cutoff"];
  fSampling = cfgd["freq_sampling"];
  soundSpeed = cfgd["sound_speed"];
  nFFT_full = cfgd["nfft_full"];
  nFFT_czt = cfgd["nfft_czt"];
  azimuthDivs = cfgd["azimuth_divs"];
  rngSNRfloor = cfgd["range_snr_floor"];
  aziSNRfloor = cfgd["azi_snr_floor"];

  #Params subject to change/ Heuristics
  nPhones = totalPhones - 1;
  azimuths = linspace(0,360,azimuthDivs)*pi/180;

  #Data preprocessing MF + CZT
  w = exp(-2im*pi*(fCeil-fFloor)/(nFFT_czt*fSampling))
  a = exp(2im*pi*fFloor/fSampling)

  chirpFile = joinpath(Pkg.dir("Caesar"),"test","testdata","template.txt");
  chirpIn = readdlm(chirpFile,',',Float64,'\n')

  #Matched Filter on Data In
  nFFT_full = nextpow(2,size(csvWaveData,1))  # MF
  mfData = zeros(Complex{Float64}, nFFT_full, totalPhones)
  mf = prepMF(chirpIn,nFFT_full,totalPhones) # MF
  mf(csvWaveData,mfData) # MF

  cztData = zeros(Complex{Float64}, nFFT_czt,totalPhones)
  filterCZT = prepCZTFilter(nFFT_full,totalPhones,w,nFFT_czt,a)
  filterCZT(mfData,cztData)

  #CBF Filter shared by Caesar
  FFTfreqs = linspace(fFloor,fCeil,nFFT_czt)
  cfgCBF_init = CBFFilterConfig(fFloor,fCeil,nFFT_czt,totalPhones,azimuths,soundSpeed, FFTfreqs)
  cfgCBF_init_LIE = CBFFilterConfig(fFloor,fCeil,nFFT_czt,nPhones,Float64[0.0;],soundSpeed, FFTfreqs)

  sas2d = SASBearing2D()
  if rangemodel == :Rayleigh
    sas2d.rangemodel = Rayleigh(100.0)
  elseif rangemodel == :Correlator
      range_mfData = zeros(Complex{Float64}, nFFT_full, totalPhones)
      range_mf = prepMF(chirpIn,nFFT_full,totalPhones) # MF
      range_mf(csvWaveData,range_mfData) # matched filter
      ranget = [0:8000-500;]*soundSpeed/fSampling
      sas2d.rangemodel = Vector{IIF.AliasingScalarSampler}(totalPhones)
      for i in 1:totalPhones
        sas2d.rangemodel[i] = IIF.AliasingScalarSampler(ranget, norm.(range_mfData[1:7501,i]), SNRfloor=rngSNRfloor)
      end
  # elseif rangemodel == :Correlator
  #   # ...
  #   range_mfData = zeros(Complex{Float64}, nFFT_full, totalPhones)
  #   range_mf = prepMF(chirpIn,nFFT_full,totalPhones) # MF
  #   range_mf(csvWaveData,range_mfData) # matched filter
  #
  #   sas2d.ranget = [0:8000-500;]*soundSpeed/fSampling
  #
  #   sas2d.rangemodel = Vector{StatsBase.ProbabilityWeights}(totalPhones)
  #   for i in 1:totalPhones
  #     safe_W = norm.(range_mfData[1:7501,i])
  #     safe_W .-= quantile(safe_W,0.4)
  #     safe_W[safe_W .< 0.0] = 0.0
  #     sas2d.rangemodel[i] = StatsBase.ProbabilityWeights(deepcopy(safe_W))
  #   end

  else
    error("No can do: what is a $rangemodel rangemodel?")
  end

  #thread safe
  sas2d.cfgTotal = cfgCBF_init
  sas2d.cfgLIE = cfgCBF_init_LIE
  sas2d.waveformsIn = cztData;
  sas2d.debugging = false;

  #create memory
  sas2d.threadreuse = Vector{SASREUSE}(Threads.nthreads())
  for thritr in 1:Threads.nthreads()
      sas2d.threadreuse[thritr] = SASREUSE()
      sas2d.threadreuse[thritr].CBFFull = zeros(Complex{Float64}, getCBFFilter2Dsize(cfgCBF_init));
      sas2d.threadreuse[thritr].CBFLIE = zeros(Complex{Float64}, getCBFFilter2Dsize(cfgCBF_init_LIE));
      sas2d.threadreuse[thritr].BFOutFull = zeros(length(azimuths))
      sas2d.threadreuse[thritr].BFOutLIE = zeros(Complex{Float64},nFFT_czt)
      sas2d.threadreuse[thritr].phaseshiftLOO = zeros(Complex{Float64},nFFT_czt)
      sas2d.threadreuse[thritr].arrayPos = zeros(Float64,totalPhones,2)
      sas2d.threadreuse[thritr].arrayPosLIE = zeros(Float64,totalPhones-1,2)

      # caching hack
      sas2d.threadreuse[thritr].hackazi = (Int[0;], Float64[0.0;])

      sas2d.threadreuse[thritr].dbg = SASDebug()
      reset!(sas2d.threadreuse[thritr].dbg)
  end

  sas2d.cfg = deepcopy(cfgd)
  sas2d.waveformsRaw = deepcopy(csvWaveData)

  sas2d
end
