using DistributedFactorGraphs

import DistributedFactorGraphs: compare


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


mutable struct SASBearing2D <: AbstractRelativeMinimize
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
  # SASBearing2D(z1) = new(z1){Float64,2}
end

# untested manifold use
getManifold(::SASBearing2D) = BearingRange_Manifold

function getSample(cfo::CalcFactor{<:SASBearing2D}, N::Int=1)
  sas2d = cfo.factor
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
                    res::AbstractVector{<:Real},
                    idx::Int,
                    meas,
                    L1,
                    XX...)
  # dx, dy, azi = 0.0, 0.0, 0.0
  # thread_data = sas2d.threadreuse[Threads.threadid()]

  if thread_data.hackazi[1][1] != idx
    for i in 1:length(XX)
      thread_data.arrayPos[i,1:2] = XX[i][1:2]-XX[1][1:2]
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

  dx, dy = L1[1]-XX[1][1] ,  L1[2]-XX[1][2]
  res[1] = (thread_data.hackazi[2][1] - atan(dy, dx))^2 +
            (meas[1] - norm([dx; dy]))^2
  nothing
end

function (cfo::CalcFactor{<:SASBearing2D})( meas,
                                            L1,
                                            XX...  )
  #
  sas2d = cfo.factor
  userdata = cfo.metadata
  idx = cfo._sampleIdx
  #
  dx, dy, azi = 0.0, 0.0, 0.0
  thread_data = sas2d.threadreuse[Threads.threadid()]
  res = [0.0;]

  if string(userdata.solvefor)[1] == 'l'
    # Solving all poses to Landmark
    # Reference the filter positions locally (first element)

    # looks like res is ignored later, but used in this function??
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
            thread_data.arrayPos[i,1:2] = XX[i][1:2]-XX[2][1:2]
            dx, dy = L1[1]-XX[2][1] ,  L1[2]-XX[2][2]
        else
            thread_data.arrayPos[i,1:2] = XX[i][1:2]-XX[1][1:2]
            dx, dy = L1[1]-XX[1][1] ,  L1[2]-XX[1][2]
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
          thread_data.arrayPos[i,1:2] = XX[i][1:2]-XX[2][1:2]
          dx, dy = L1[1]-XX[2][1] ,  L1[2]-XX[2][2]
      else
          thread_data.arrayPos[i,1:2] = XX[i][1:2]-XX[1][1:2]
          dx, dy = L1[1]-XX[1][1] ,  L1[2]-XX[1][2]
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

function DFG.compare(a::SASDebug,b::SASDebug)
  TP = true
  TP = TP && a.beams == b.beams
  TP = TP && a.azi_smpls == b.azi_smpls
  return TP
end


mutable struct PackedSASBearing2D <: DFG.AbstractPackedFactor
    rangemodel::String
    totalPhones::Int
    wavedataRawV::Vector{Float64}
    wavedataRawV_dim::Int
    cfgJson::String
    cfgTotal::String
    cfgLIE::String
    waveformsInReal::Vector{Float64}
    waveformsInIm::Vector{Float64}
    PackedSASBearing2D() = new()
    PackedSASBearing2D(rangemodel::String, tp::Int, wavedataRawV::Vector{Float64}, wavedataRawV_dim::Int, cfgJson::String, cfgTotal::String, cfgLIE::String, waveformsInReal::Vector{Float64}, waveformsInIm::Vector{Float64}) = new(rangemodel, tp, wavedataRawV, wavedataRawV_dim, cfgJson, cfgTotal, cfgLIE, waveformsInReal, waveformsInIm)
    PackedSASBearing2D(rangemodel::String, tp::Int, wd::Array{Float64,2}, cf::Dict, cfgTotal::String, cfgLIE::String, waveformsInReal::Vector{Float64}, waveformsInIm::Vector{Float64}) = new(rangemodel, tp, wd[:], size(wd,1), JSON2.write(cf), cfgTotal, cfgLIE, waveformsInReal, waveformsInIm)
end

# Note: Need this otherwise it uses a default converter and causes type conversion issues.
# Safest to just do it here to guarantee it's done.
import Base.convert

function convert(::Type{<:PackedSASBearing2D}, d::SASBearing2D)
  # range model is vector, packing it cleanly
  rangeString = ""
  if typeof(d.rangemodel) == Vector{AliasingScalarSampler}
    rangeString = JSON.json(map(rm -> string(rm), d.rangemodel))
  else
    error("Can't pack range model of type $(typeof(d.rangemodel)) yet.")
  end

  cfgTotalJson = JSON2.write(d.cfgTotal)
  cfgLIEJson = JSON2.write(d.cfgLIE)
  waveformsInReal = map(r -> real(r), d.waveformsIn[:])
  waveformsInIm = map(r -> imag(r), d.waveformsIn[:])
  totalPhones = size(d.waveformsRaw,2)
  pPacked = PackedSASBearing2D(rangeString, totalPhones, d.waveformsRaw, d.cfg, cfgTotalJson, cfgLIEJson, waveformsInReal, waveformsInIm)
  return pPacked
end

function convert(::Type{SASBearing2D}, d::PackedSASBearing2D)::SASBearing2D
  rangePacked = JSON2.read(d.rangemodel)
  rangeModel = map(r -> extractdistribution(r), rangePacked)
  cfgJson = JSON2.read(d.cfgJson, Dict)
  cfgTotal = JSON2.read(d.cfgTotal, CBFFilterConfig)
  cfgLIE = JSON2.read(d.cfgLIE, CBFFilterConfig)
  waveformsIn = map(i -> Complex{Float64}(d.waveformsInReal[i], d.waveformsInIm[i]), 1:(length(d.waveformsInReal)))
  waveformsIn = reshape(waveformsIn, Int(length(waveformsIn)/d.totalPhones), :)

  sas2d = SASBearing2D()
  sas2d.rangemodel = rangeModel
  sas2d.cfg = cfgJson
  sas2d.cfgTotal = cfgTotal
  sas2d.cfgLIE = cfgLIE
  sas2d.waveformsIn = waveformsIn
  sas2d.debugging = false
  sas2d.waveformsRaw = reshape(d.wavedataRawV, d.wavedataRawV_dim, :)
  prepareSASMemory(sas2d)

  return sas2d
end


function DFG.compare(a::SASBearing2D, b::SASBearing2D)
  TP = true
  TP &= compare(a.cfgTotal, b.cfgTotal)               # CBFFilterConfig
  @debug "cfgTotal: $(compare(a.cfgTotal, b.cfgTotal))"
  TP &= compare(a.cfgLIE, b.cfgLIE)                   # CBFFilterConfig
  @debug "cfgLIE: $(compare(a.cfgLIE, b.cfgLIE))"
  TP &= a.waveformsIn == b.waveformsIn         # Array{Complex{Float64}}
  @debug "waveformsIn: $(a.waveformsIn == b.waveformsIn)"

  # Rangemodel
  # Utility dist formula
  calcArrayDist = (a,b, percAcceptable=1) -> begin
    sum(a) == 0 && sum(b) == 0 && return false
    sum(a) == 0 && return false
    sum(abs.(a-b))/sum(abs.(a)) * 100.0 < percAcceptable && return true
    return false
  end
  TP &= typeof(a.rangemodel) == typeof(b.rangemodel)
  @debug "typeof(rangemodel): $((typeof(a.rangemodel) == typeof(b.rangemodel)))"
  if typeof(a.rangemodel) == typeof(b.rangemodel) == Array{AliasingScalarSampler,1}
    TP &= length(a.rangemodel) == length(b.rangemodel)
    @debug "rangemodel (length): $(length(a.rangemodel) == length(b.rangemodel))"

    if length(a.rangemodel) == length(b.rangemodel)
      for i in 1:length(a.rangemodel)
        # Domain
        TP &= calcArrayDist(a.rangemodel[i].domain, b.rangemodel[i].domain)
        @debug "rangemodel (model[$i].domain): $(calcArrayDist(a.rangemodel[i].domain, b.rangemodel[i].domain))"
        # Weights
        TP &= calcArrayDist(a.rangemodel[i].weights, b.rangemodel[i].weights)
        @debug "rangemodel (model[$i].weights): $(calcArrayDist(a.rangemodel[i].weights, b.rangemodel[i].weights))"
      end
    end
  elseif typeof(a.rangemodel) == typeof(b.rangemodel) == Rayleigh
    TP &= a.rangemodel == b.rangemodel
    @debug "rangemodel (Raleigh): $(a.rangemodel == b.rangemodel)"
  end

  TP &= a.cfg == b.cfg                         # Dict
  @debug "cfg: $(a.cfg == b.cfg)"
  TP &= a.waveformsRaw == b.waveformsRaw       # Array{Float64,2}
  @debug "waveformsRaw: $(a.waveformsRaw == b.waveformsRaw)"
  TP &= a.debugging == b.debugging                       # SASDebug
  @debug "debugging: $(a.debugging == b.debugging)"
  return TP
end
