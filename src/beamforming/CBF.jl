# Conventional Beamformer (CBF)
# Construct filter before beamforming

mutable struct CBFFilterConfig
  fFloor::Float64
  fCeil::Float64
  dataLen::Int
  nPhones::Int
  azimuths::Array{Float64}
  soundSpeed::Float64
  FFTfreqs::Array{Float64}
end

function compare(a::CBFFilterConfig,b::CBFFilterConfig)
  TP = true
  TP = TP && a.fFloor == b.fFloor       # Float64
  TP = TP && a.fCeil == b.fCeil       # Float64
  TP = TP && a.dataLen == b.dataLen       # Int
  TP = TP && a.nPhones == b.nPhones       # Int
  TP = TP && a.azimuths == b.azimuths       # Array{Float64}
  TP = TP && a.soundSpeed == b.soundSpeed       # Float64
  TP = TP && a.FFTfreqs == b.FFTfreqs
  TP
end

function getCBFFilter2Dsize(thisCfg::CBFFilterConfig)
    return (length(thisCfg.azimuths),thisCfg.dataLen,thisCfg.nPhones)
end

function constructCBFFilter2D!(thisCfg::CBFFilterConfig,arrayPos::Array,filterOut::Array,sourceXY::Array, dataTemp::Array, delaysTemp::Array)

    #dataTemp = zeros(Complex{Float64}, thisCfg.nPhones,thisCfg.dataLen)

    # combination of fCeil and fSampling limit information content extracted by thirsCfg.azimuths. Sample faster to get higher res BF
    for iter in eachindex(thisCfg.azimuths)
        @inbounds sourceXY[1] = -cos(thisCfg.azimuths[iter])
        @inbounds sourceXY[2] = -sin(thisCfg.azimuths[iter])
        #tDelays[:] = (arrayPos * sourceXY)/thisCfg.soundSpeed

        for piter in 1:thisCfg.nPhones
            @inbounds dt = (arrayPos[piter,1]*sourceXY[1] + arrayPos[piter,2]*sourceXY[2])/thisCfg.soundSpeed

            @inbounds dataTemp[piter,:] = exp.(delaysTemp.*-2im*pi*dt)
            #for fftiter in 1:thisCfg.dataLen
            #     @time dataTemp[piter,fftiter] = exp(-2im*pi*dt*thisCfg.FFTfreqs[fftiter])
            #end
        end

        # @time filterOut[iter,:,:] = transpose(conj!(exp.(-2im*pi*tDelays*FFTfreqs'))) #conjugate transpose in place
        @inbounds adjoint!(view(filterOut,iter,:,:),dataTemp)
    end
    nothing
end

function CBF2D_DelaySum!(thisCfg::CBFFilterConfig,dataIn::Array,dataOut::Array,temp1::Array,temp2::Array,thisFilter::Array; dochecks::Bool=false)
    if dochecks
      if size(dataIn,1) !=thisCfg.dataLen || size(dataIn,2)!=thisCfg.nPhones
          error("Data Array wrong dimensions")
      end
      if size(thisFilter)!=(length(thisCfg.azimuths),thisCfg.dataLen,thisCfg.nPhones)
          error("Provided Filter does not match provided config.")
      end
    end
    # minimal syntax
#    for iter in eachindex(thisCfg.azimuths)
#        dataOut[iter] = sum(norm.(sum(dataIn.*thisFilter[iter,:,:],2),2),1)[1]
#    end

    for iter in eachindex(thisCfg.azimuths)
        # for diter in 1:thisCfg.dataLen
        #      for phiter in 1:thisCfg.nPhones
        #          temp2[diter,phiter] = dataIn[diter,phiter]
        #      end
        #  end
        copyto!(temp2,dataIn)
        #temp2 = deepcopy(dataIn) # think somethign not zeroing properly?
        @fastmath temp2 .*= @view thisFilter[iter,:,:]
        @fastmath @inbounds sum!(temp1,temp2)

        @fastmath @inbounds @simd for ditr in 1:thisCfg.dataLen
            temp1[ditr] = norm(temp1[ditr])
        end

        @inbounds dataOut[iter] = sum(temp1)
        #dataOut[iter] = sum(norm.(temp1,2))
    end

    # dataOut = deepcopy(dataOut)

    # brute force in-place syntax
    # fill!(dataOut, 0.0)
    # phones_combined_ = zeros(thisCfg.dataLen) .+ im*0.0
    # for iter in eachindex(thisCfg.azimuths)
    #   fill!(phones_combined_, 0.0)
    #   for diter in 1:thisCfg.dataLen
    #     for phiter in 1:thisCfg.nPhones
    #       phones_combined_[diter] += dataIn[diter,phiter]*thisFilter[iter,diter,phiter]
    #     end
    #     dataOut[iter] += norm(phones_combined_[diter],2)
    #   end
    # end
    # @assert norm(dataOut - dataOutTest) < 0.00001

    # TODO - some hybrid clean in-place syntax
    nothing
end





"""
    $(TYPEDSIGNATURES)

Phase shifting ztransWave contents of leave one out element of `elemPositions` (positions of array elements)
based on dx,dy position perturbation.  The ztransWave is chirp z-transform of LOO elements's waveform data,
and only considering subset of frequencies in ztransWave (i.e. smaller than size of raw waveform data)
"""
function phaseShiftSingle!(sourceXY::Vector{Float64},
                           thisCfg::CBFFilterConfig,
                           azi::R,
                           positions::Array,
                           ztransWave::Array{Complex{R2}}  ) where {R <: Real, R2 <: Real}

  sourceXY[1] = -cos(azi) # unit vector in azimuth direction
  sourceXY[2] = -sin(azi)
  dt = (positions' * sourceXY)/thisCfg.soundSpeed
  ztransWave .*= conj(exp.(-2im*pi*dt*thisCfg.FFTfreqs))

  # @assert thisCfg.dataLen == length(ztransWave)
  # for j in 1:thisCfg.dataLen
  #    ztransWave[j] *= conj(exp(-2im*pi*dt*thisCfg.FFTfreqs[j]))
  # end
  nothing
end

"""
    $(TYPEDSIGNATURES)

Generate the incoming waveform (into `bfOutLIEl`) at the desired look angle `aziInd`.
"""
function liebf!(bfOutLIEl, cztWaveLIEl, bfFilterLIEl, aziIndl, temp; normalize::Bool=false)

  # perform conventional beam forming with single look angle (on leave-ins)
  #bfOutLIEl[:] = sum(cztWaveLIEl.*bfFilterLIEl[aziIndl,:,:],2)

  copy!(temp,cztWaveLIEl)
  temp .*= @view bfFilterLIEl[aziIndl,:,:]
  sum!(bfOutLIEl,temp)

  # OPTIONAL normalize according to number of leave in elements
  # REQUIRED when doing residual rather than corr of waveform (measured - predict)
  if normalize
    bfOutLIEl ./= size(cztWaveLIEl,2) # corrWaveLIEl
  end

  # predict time series of beam formed output of leave-in elements
  # ifft!(bfOutLIE)
  nothing
end




#
