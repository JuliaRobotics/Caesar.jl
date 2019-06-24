# Chirp Z-Transform

mutable struct CZTFilter{T}
    dataLen::Int #Expected data length
    nPhones::Int #Expected number of phones
    fftIter::Vector{Int} #preallocated vector for fft iteration
    phonesIter::Vector{Int} #preallocated vector for phones iteration
    outLen::Int #Output length
    wk2::Vector{Complex{T}}
    Awk2::Vector{Complex{T}}
    Fwk2::Vector{Complex{T}}
    tempData::Array{Complex{T}}
    nFFT::Int
    CZTFilter{T}() where T = new{T}()
end

function (czt::CZTFilter{T})(dataIn::Array, dataOut::Array{Complex{T}}) where {T <: Real}
    dataLen = size(dataIn,1)
    if dataLen!=czt.dataLen
        error("Data Signal length: $dataLen Filter expected: $(czt.dataLen)")
    end
    if czt.nPhones != size(dataIn,2)
        error("Number of phones: $(size(dataIn,2)). Expected: $(czt.nPhones)")
    end
    vdI = view(dataIn,1:dataLen,:)
    czt.tempData[:] .= 0.0 + 0.0im
    czt.tempData[1:dataLen,:] = vdI .* czt.Awk2; #zero out temp data
    fft!(czt.tempData,1)
    # fwk = czt.Fwk2
    for j in czt.phonesIter
      czt.tempData[:,j] .*= czt.Fwk2
    end
    ifft!(czt.tempData,1)
    vtD = view(czt.tempData, czt.dataLen:(czt.dataLen+czt.outLen-1),:)
    copyto!(dataOut, vtD)
    dataOut .*= czt.wk2
    nothing
end

# function (czt::CZTFilter{T})(dataIn::Array, dataOut::Array{Complex{T}}) where {T <: Real}
#     dataLen = size(dataIn,1)
#     if dataLen!=czt.dataLen
#         error("Data Signal length: $dataLen Filter expected: $(czt.dataLen)")
#     end
#     if czt.nPhones != size(dataIn,2)
#         error("Number of phones: $(size(dataIn,2)). Expected: $(czt.nPhones)")
#     end
#     for i in 1:czt.dataLen
#         for j in czt.phonesIter
#             czt.tempData[i,j] = dataIn[i,j] * czt.Awk2[i]
#         end
#     end
#     fft!(czt.tempData,1)
#     for i in czt.fftIter
#         for j in czt.phonesIter
#           czt.tempData[i,j] = czt.tempData[i,j] * czt.Fwk2[i]
#         end
#     end
#     ifft!(czt.tempData,1)
#     for i in czt.dataLen:(czt.dataLen+czt.outLen-1)
#       for j in czt.phonesIter
#         dataOut[i-czt.dataLen+1,j] = czt.tempData[i,j]*czt.wk2[i-czt.dataLen+1]
#       end
#     end
#     nothing
# end


#orig - starting point in the plane
#theoretically allocating all memory here
function prepCZTFilter(n::Int, nPhones::Int, w::Complex{T}, m::Int=-1, a::Complex{T}=1.0+0im) where {T}
    # , T::Type=Float64)
    out = CZTFilter{T}()
    out.dataLen=n
    if m<0
        m=n
    end
    out.outLen=m
    out.nPhones = nPhones
    out.phonesIter = 1:out.nPhones #linspace(1,nPhones,nPhones)
    out.nFFT = nextpow(2,n+m)
    out.fftIter = 1:out.nFFT # linspace(1,out.nFFT,out.nFFT)
    out.tempData = zeros(Complex{T},out.nFFT,nPhones)
    k = range(0,stop=max(n,m)-1,length=max(n,m))
    #k = linspace(0,max(n,m)-1,max(n,m))
    wk2 = w.^(k.^2/2); #@show size(wk2)
    out.Awk2 = ((a.^-k) .* wk2)[1:n]
    out.Fwk2 = zeros(Complex{T},out.nFFT)
    out.Fwk2[1:n] = 1.0 ./wk2[n:-1:1]
    out.Fwk2[n:m+n-1] = 1.0 ./wk2[1:m]
    #out.Fwk2[1:n+1] = 1./append!(wk2[n:-1:2], wk2[1:m])    #alternative for two lines above
    fft!(out.Fwk2)
    out.wk2 = wk2[1:m]
    return out
end







#
