

function sizeforfft(len::Int, T::DataType=Float64)
    len2 = nextpow(2,len)
    zeros(Complex{T}, len2)
end

mutable struct PaddedFFT{T} <: Function
    U::Vector{Complex{T}}
    PaddedFFT(n::Int, D=Float64) = n > 0 ? new{D}(sizeforfft(n, D)) : error("n=$n must be than 0.")
    PaddedFFT{T}(v::Vector{Complex{T}}) where {T <: Union{Float32, Float64}} = new{T}(sizeforfft(length(v), T))
end

function (pfft::PaddedFFT{T})(u2::Vector{Complex{T}}) where {T <: Real}
    len = length(u2)
    fill!(pfft.U, 0+0*im)
    @inbounds pfft.U[1:len] = u2
    fft!(pfft.U)
    nothing
end

# using BenchmarkTools
# u2 = randn(2^20 - 20000) .+ 0im;
# ffft64 = PaddedFFT{Float64}(u2)
# size(ffft64.U)
# @btime ffft64(u2)
# ffft64(u2)
# u32 = randn(Float32, 2^20 - 20000) .+ 0im;
# ffft32 = PaddedFFT{Float32}(length(u32),Float32)
# @btime ffft32(u32)
