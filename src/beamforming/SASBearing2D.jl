
mutable struct SASDebug
  beams::Vector{Vector{Float64}}
  azi_smpls::Vector{Float64}
  SASDebug() = new()
end

mutable struct SASREUSE
    CBFFull::Array{Complex{Float64}}
    CBFLIE::Array{Complex{Float64}}
    BFOutFull::Array{Complex{Float64}}
    BFOutLIE::Array{Complex{Float64}}
    phaseshiftLOO::Array{Complex{Float64}}
    arrayPos::Array{Float64}
    arrayPosLIE::Array{Float64}
    hackazi::Tuple{Vector{Int}, Vector{Float64}} # [idx], [azi]

    dbg::SASDebug
    SASREUSE() = new()
end


mutable struct SASBearing2D <: IncrementalInference.FunctorPairwiseMinimize
  cfgTotal::CBFFilterConfig
  cfgLIE::CBFFilterConfig
  waveformsIn::Array{Complex{Float64}}

  # to allow multithreaded behaviour
  threadreuse::Vector{SASREUSE}

  rangemodel::Union{Distributions.Rayleigh, Vector{IIF.AliasingScalarSampler}} # , Vector{<:StatsBase.AbstractWeights}
  ranget::Vector{Float64}
  cfg::Dict
  waveformsRaw::Array{Float64,2}
  debugging::Bool

  SASBearing2D() = new()
  # SASBearing2D(z1) = new(z1)
end
function getSample(sas2d::SASBearing2D, N::Int=1)
  # TODO bring the solvefor index tp getSample function
  if typeof(sas2d.rangemodel) == Distributions.Rayleigh
    return (reshape(rand(sas2d.rangemodel, N),1,N), )
  else
    # TODO have to use the first waveform, since we do not know who is being solved for
    return (reshape(rand(sas2d.rangemodel[1], N), 1,N), )
  end
  #   smps = zeros(N)
  #   StatsBase.alias_sample!(sas2d.ranget, sas2d.rangemodel[1], smps);
  #   return (smps, )
end
function (sas2d::SASBearing2D)(
            res::Array{Float64},
            userdata::FactorMetadata,
            idx::Int,
            meas::Tuple,
            L1::Array{Float64,2},
            XX...  )

  dx, dy, azi = 0.0, 0.0, 0.0
  thread_data = sas2d.threadreuse[Threads.threadid()]

  if string(userdata.solvefor)[1] == 'l'
    # Solving all poses to Landmark
    # Reference the filter positions locally (first element)
    for i in 1:length(XX)
      thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[1][1:2,idx]
    end
    if thread_data.hackazi[1][1] != idx
      constructCBFFilter2D!(sas2d.cfgTotal, thread_data.arrayPos, thread_data.CBFFull)
      CBF2D_DelaySum!(sas2d.cfgTotal, sas2d.waveformsIn, thread_data.BFOutFull, thread_data.CBFFull)

      # # TODO -- move to specialized getSample
      thread_data.hackazi[1][1] = idx

      azi_weights = (repmat(norm.(thread_data.BFOutFull), 1,8)')[:]
      azi_domain = collect(linspace(0,2pi-1e-10,length(azi_weights)))
      bss = AliasingScalarSampler(azi_domain, azi_weights, SNRfloor=sas2d.cfg["azi_snr_floor"])

      thread_data.hackazi[2][1] = wrapRad(rand(bss)[1])

      if sas2d.debugging
          push!(sas2d.dbg.beams, deepcopy(sas2d.BFOut))
          push!(sas2d.dbg.azi_smpls, thread_data.hackazi[2][1])
      end
    end

    dx, dy = L1[1,idx]-XX[1][1,idx] ,  L1[2,idx]-XX[1][2,idx]
    res[1] = (thread_data.hackazi[2][1] - atan2(dy, dx))^2 +
             (meas[1][idx] - norm([dx; dy]))^2

  else
      # WORK IN PROGRESS
    workvarlist = userdata.variablelist[2:end] # skip landmark element
    looelement = findfirst(workvarlist .== userdata.solvefor)
    # @assert looelement > 0
    # looelement = parse(Int, string(userdata.solvefor)[2:end])
    elementset = setdiff(Int[1:length(workvarlist);], [looelement;])

    # assume you get variable list [:l1,:x8,:x9,:x10]
    # looelement must be int index [1..numelemnts]
    # elementset is Vector of int indices for LIE, eg. [1;2;4;5], and LOO is 3


    # Reference first or second element
    for i in 1:length(XX)
        if looelement==1
            thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[2][1:2,idx]
            dx, dy = L1[1,idx]-XX[2][1,idx] ,  L1[2,idx]-XX[2][2,idx]
        else
            thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[1][1:2,idx]
            dx, dy = L1[1,idx]-XX[1][1,idx] ,  L1[2,idx]-XX[1][2,idx]
        end
    end
    thread_data.arrayPosLIE[1:end,1:2] = thread_data.arrayPos[elementset,1:2]
    azi = atan2(dy,dx)
    sas2d.cfgLIE.azimuths = [azi;]
    # @show sas2d.cfgLIE.nPhones, size(sas2d.cfgLIE.arrayPos), size(sas2d.CBFLIE)
    constructCBFFilter2D!(sas2d.cfgLIE, thread_data.arrayPosLIE, thread_data.CBFLIE)
    liebf!(thread_data.BFOutLIE, sas2d.waveformsIn[:,elementset], thread_data.CBFLIE, 1, normalize=true)

    thread_data.phaseshiftLOO = deepcopy(sas2d.waveformsIn[:,looelement])
    phaseShiftSingle!(sas2d.cfgLIE, azi, thread_data.arrayPos[looelement,1:2], thread_data.phaseshiftLOO)

    bfOutLIEcorr = thread_data.BFOutLIE + thread_data.phaseshiftLOO
    res[1] = -sum(norm.(bfOutLIEcorr))
  end
  res[1]
end





function reset!(dbg::SASDebug)
  dbg.beams = [Float64[];]
  dbg.azi_smpls = Float64[]
  nothing
end

function compare(a::SASDebug,b::SASDebug)
  TP = true
  TP = TP && a.beams == b.beams
  TP = TP && a.azi_smpls == b.azi_smpls
  return TP
end


mutable struct PackedSASBearing2D <: IncrementalInference.PackedInferenceType
    totalPhones::Int
    wavedataRawV::Vector{Float64}
    wavedataRawV_dim::Int
    cfg_json::String
    PackedSASBearing2D() = new()
    PackedSASBearing2D(tp::Int, wd::Array{Float64,2}, cf::Dict) = new(tp, wd[:], size(wd,1), JSON.json(cf) )
end

function convert(::Type{PackedSASBearing2D}, d::SASBearing2D)
  PackedSASBearing2D(size(d.waveformsRaw,2), d.waveformsRaw, d.cfg)
end

function convert(::Type{SASBearing2D}, d::PackedSASBearing2D)
  prepareSAS2DFactor(d.totalPhones, reshape(d.wavedataRawV,d.wavedataRawV_dim,:), cfgd=JSON.parse(d.cfg_json))
end

function compare(a::SASBearing2D, b::SASBearing2D)
  TP = true
  TP = TP && compare(a.cfgTotal, b.cfgTotal)               # CBFFilterConfig
  TP = TP && compare(a.cfgLIE, b.cfgLIE)                   # CBFFilterConfig
  TP = TP && a.filterCBFTotal == b.filterCBFTotal   # Array{Complex{Float64}}
  TP = TP && a.CBFLIE == b.CBFLIE       # Array{Complex{Float64}}
  TP = TP && a.waveformsIn == b.waveformsIn         # Array{Complex{Float64}}
  TP = TP && a.BFOut == b.BFOut                     # Array{Complex{Float64}}
  TP = TP && a.BFOutLIE == b.BFOutLIE               # Array{Complex{Float64}}
  TP = TP && a.phaseshiftLOO == b.phaseshiftLOO     # Array{Complex{Float64}}
  TP = TP && a.hackazi[1] == b.hackazi[1]           # Tuple{Vector{Int}, Vector{Float64}}
  TP = TP && a.hackazi[2] == b.hackazi[2]
  TP = TP && a.rangemodel.σ == b.rangemodel.σ       # Distributions.Rayleigh
  TP = TP && compare(a.dbg, b.dbg)                       # SASDebug
  TP = TP && a.cfg == b.cfg                         # Dict
  TP = TP && a.waveformsRaw == b.waveformsRaw       # Array{Float64,2}
  TP
end
