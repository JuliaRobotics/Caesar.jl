
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
    BFOutLIETemp::Array{Complex{Float64}}
    phaseshiftLOO::Array{Complex{Float64}}
    arrayPos::Array{Float64}
    arrayPosLIE::Array{Float64}
    hackazi::Tuple{Vector{Int}, Vector{Float64}} # [idx], [azi]
    oncebackidx::Vector{Int} # [idx]

    workvarlist::Vector{Symbol}
    looelement::Int
    elementset::Vector{Int}
    waveformsLIE::Array{Complex{Float64},2}
    waveformsLOOc::Array{Complex{Float64},1}

    BFtemp::Array{Complex{Float64}}

    LIEtemp::Array{Complex{Float64}}
    temp1::Array{Complex{Float64},1}
    temp2::Array{Complex{Float64},2}

    temp::Array{Complex{Float64}}
    sourceXY::Vector{Float64}

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
  if isa(sas2d.rangemodel, Distributions.Rayleigh)
    return (reshape(rand(sas2d.rangemodel, N),1,N), )
  else
    # TODO have to use the first waveform, since we do not know who is being solved for
    return (reshape(rand(sas2d.rangemodel[1], N), 1,N), )
  end
end

function forwardsas(sas2d::SASBearing2D,
                    thread_data::SASREUSE,
                    res::Vector{Float64},
                    idx::Int,
                    meas::Tuple,
                    L1::Array{Float64,2},
                    XX...)
  # dx, dy, azi = 0.0, 0.0, 0.0
  # thread_data = sas2d.threadreuse[Threads.threadid()]

  if thread_data.hackazi[1][1] != idx
    for i in 1:length(XX)
      thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[1][1:2,idx]
    end
    constructCBFFilter2D!(sas2d.cfgTotal, thread_data.arrayPos, thread_data.CBFFull,thread_data.sourceXY, thread_data.BFtemp)

    #fill!(thread_data.temp1,0)
    #fill!(thread_data.temp2,0)

    CBF2D_DelaySum!(sas2d.cfgTotal, sas2d.waveformsIn, thread_data.BFOutFull,thread_data.temp1,thread_data.temp2,thread_data.CBFFull)

    thread_data.hackazi[1][1] = idx

    azi_weights = (repeat(norm.(thread_data.BFOutFull), 1,8)')[:]
    azi_domain = collect(LinRange(0,2pi-1e-10,length(azi_weights)))
    bss = AliasingScalarSampler(azi_domain, azi_weights, SNRfloor=sas2d.cfg["azi_snr_floor"])

    thread_data.hackazi[2][1] = wrapRad(rand(bss)[1])

    if sas2d.debugging
        push!(thread_data.dbg.beams, deepcopy(thread_data.BFOutFull))
        push!(thread_data.dbg.azi_smpls, thread_data.hackazi[2][1])
    end
  end

  dx, dy = L1[1,idx]-XX[1][1,idx] ,  L1[2,idx]-XX[1][2,idx]
  res[1] = (thread_data.hackazi[2][1] - atan(dy, dx))^2 +
           (meas[1][idx] - norm([dx; dy]))^2
  nothing
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

    forwardsas(sas2d, thread_data, res, idx,meas,L1,XX...)

  else

    if thread_data.oncebackidx[1] != idx
      # ccall(:jl_, Void, (Any,), "idx=$(idx)\n")
      thread_data.oncebackidx[1] = idx
      #run for eacn new idx
      thread_data.workvarlist = userdata.variablelist[2:end] # skip landmark element
      thread_data.looelement = findfirst(thread_data.workvarlist .== userdata.solvefor)
      thread_data.elementset = setdiff(Int[1:length(thread_data.workvarlist);],   [thread_data.looelement;])

      # Reference first or second element
      for i in 1:length(XX)
        if thread_data.looelement==1
            thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[2][1:2,idx]
            dx, dy = L1[1,idx]-XX[2][1,idx] ,  L1[2,idx]-XX[2][2,idx]
        else
            thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[1][1:2,idx]
            dx, dy = L1[1,idx]-XX[1][1,idx] ,  L1[2,idx]-XX[1][2,idx]
        end
      end

      for i in 1:sas2d.cfgLIE.dataLen
          for j in 1:length(thread_data.elementset)
            thread_data.waveformsLIE[i,j] = sas2d.waveformsIn[i,thread_data.elementset[j]]
          end
          thread_data.waveformsLOOc[i] = sas2d.waveformsIn[i,thread_data.looelement]
      end



    end #end of idx

    # Reference first or second element
    for i in 1:length(XX)
      if thread_data.looelement==1
          thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[2][1:2,idx]
          dx, dy = L1[1,idx]-XX[2][1,idx] ,  L1[2,idx]-XX[2][2,idx]
      else
          thread_data.arrayPos[i,1:2] = XX[i][1:2,idx]-XX[1][1:2,idx]
          dx, dy = L1[1,idx]-XX[1][1,idx] ,  L1[2,idx]-XX[1][2,idx]
      end
    end

    for i in 1:length(thread_data.elementset), j in 1:2
        elem = thread_data.elementset[i]
        thread_data.arrayPosLIE[i,j] = thread_data.arrayPos[elem,j]
    end
    azi = atan(dy,dx)
    sas2d.cfgLIE.azimuths = [azi;]


    constructCBFFilter2D!(sas2d.cfgLIE, thread_data.arrayPosLIE, thread_data.CBFLIE,thread_data.sourceXY,thread_data.LIEtemp)

    liebf!(thread_data.BFOutLIE, thread_data.waveformsLIE, thread_data.CBFLIE, 1, thread_data.temp, normalize=true)

    copy!(thread_data.phaseshiftLOO,thread_data.waveformsLOOc)
    phaseShiftSingle!(thread_data.sourceXY, sas2d.cfgLIE, azi, thread_data.arrayPos[thread_data.looelement,1:2], thread_data.phaseshiftLOO)

    res[1] = 0.0
    copy!(thread_data.BFOutLIETemp,thread_data.BFOutLIE)
    @fastmath @inbounds @simd for i in 1:length(thread_data.phaseshiftLOO)
      thread_data.BFOutLIETemp[i] += thread_data.phaseshiftLOO[i]
      res[1] -= norm(thread_data.BFOutLIETemp[i])
    end
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
