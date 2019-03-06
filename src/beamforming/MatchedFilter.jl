#Implements a Matched Filter for BEAMFORMING

mutable struct MatchedFilter
    filter::Array
    chirpF::Array{Complex{Float64}}
    nFFT::Int
    nPhones::Int
    MatchedFilter() = new()
end
function (mf::MatchedFilter)(dataIn::Array,dataOut::Array)
    if size(dataOut)!=(mf.nFFT,mf.nPhones) error("Data Out has the wrong dims") end
    dataOut[1:size(dataIn,1),:] = dataIn
    fft!(dataOut,1)
    dataOut ./= (abs.(dataOut).+1e-9) # OPTIONAL PHAT transform
    dataOut .*= mf.filter
    ifft!(dataOut,1)
    nothing
end

function prepMF(chirpIn::Array,n::Int,m::Int)
    if n<length(chirpIn)
        error("FFT range smaller than chirp")
    end
    thisMF = MatchedFilter()
    thisMF.nFFT=n
    thisMF.nPhones=m
    thisMF.chirpF = zeros(Complex{Float64},n)
    thisMF.chirpF[1:length(chirpIn)] = chirpIn
    conj!(fft!(thisMF.chirpF))
    thisMF.filter = repeat(thisMF.chirpF,1,m) # repmat
    return thisMF
end





#
