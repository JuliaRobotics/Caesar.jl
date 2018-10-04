#####
# 1. We will have to write protobuf distribution packers/unpackers for all the supported distribution types.
# 2. We will have to write protobuf factor packers/unpackers for all the supported factor types.
####

using ProtoBuf
using RoME, Distributions
using Base.convert

# Build a module of all the packed distributions and packed factors
module Protos
    using ProtoBuf
    # Distributions
    files = readdir(joinpath(Pkg.dir("RoME"), "protos", "distributions"))
    filter!(f -> endswith(f, ".jl"), files)
    foreach(f -> include(joinpath(Pkg.dir("RoME"), "protos", "distributions", f)), files)
    # Factors
    files = readdir(joinpath(Pkg.dir("RoME"), "protos", "factors"))
    filter!(f -> endswith(f, ".jl"), files)
    foreach(f -> include(joinpath(Pkg.dir("RoME"), "protos", "factors", f)), files)
end

### START Distribution Packers/Unpackers

## Normal
function protoPackDistribution(dist::Distributions.Normal)::Protos.Normal
    return Protos.Normal(;mean=dist.μ, std=dist.σ)
end
function protoUnpackDistribution(packedDist::Protos.Normal)::Distributions.Normal
    return Distributions.Normal(packedDist.mean, packedDist.std)
end

### END Distribution Packers/Unpackers

function protoEncodeDistribution(dist::T)::Protos.PackedDistribution where {T <: Distributions.Distribution}
    baseName = replace(string(typeof(dist).name), "Distributions.", "")
    unpackedName = string(typeof(dist).name)
    packedName = "Protos."*baseName
    info("Encoding $unpackedName into $packedName...")

    # 1. Convert to packed type
    packedDist = nothing
    try
        @show packedDist = protoPackDistribution(dist)
    catch ex
        showerror(STDERR, er, catch_backtrace())
        error("Unable to find packing type '$(packedName)' for distribution '$(typeof(dist))'. Have you written converters for this distribution?")
    end

    # 2. Encode it.
    ob = PipeBuffer()
    writeproto(ob, packedDist)
    packedDist = Protos.PackedDistribution(; distType=baseName, packed=ob.data)
    return packedDist
end

function protoDecodeDistribution(pDist::Protos.PackedDistribution)::Distributions.Distribution
    packedName = "Protos."*pDist.distType
    unpackedName = "Distributions."*pDist.distType
    info("Decoding $packedName into $unpackedName...")

    # 1. Decode it using an instance of the packed name
    packedDist = nothing
    try
        ib = PipeBuffer(pDist.packed)
        #TODO: getfield on module.packedname to create instance 
        packedDist = readproto(ib, eval(parse(packedName*"()"))) #Use parse and eval to make an instance - little vodoo magickry :/
    catch er
        showerror(STDERR, er, catch_backtrace())
        error("Unable to decode Protobuf into $packedName. Please check Protobuf structure for this type. Error = $(er).")
    end

    # 2. Convert to full type
    unpackedType = nothing
    try
        unpackedType = protoUnpackDistribution(packedDist)
    catch er
        showerror(STDERR, er, catch_backtrace())
        error("Unable to find convert Protobuf deserialization type $(packedDist) to $unpackedName. Please check distribution conversion tests. Error = $(er).")
    end
    return unpackedType
end

### START Factor Packers/Unpackers

function protoPackFactor(d::Pose2Point2BearingRange)::Protos.PackedPose2Point2BearingRange
    return Protos.PackedPose2Point2BearingRange(; bearing=protoEncodeDistribution(d.bearing), range=protoEncodeDistribution(d.range))
end
function protoUnpackFactor(d::Protos.PackedPose2Point2BearingRange)::Pose2Point2BearingRange
    return Pose2Point2BearingRange(protoDecodeDistribution(d.bearing), protoDecodeDistribution(d.range))
end

### END Factor Packers/Unpackers

############# Testing #############

# Building our factor.
nBearing = Normal(-pi/3,0.05)
nRange = Normal(5, 5)
p2br = Pose2Point2BearingRange(nBearing,nRange)

# Converting it to packed.
packedp2br = protoPackFactor(p2br)
p2brBack = protoUnpackFactor(packedp2br)

# Confirming all is good.
p2brBack.bearing == p2br.bearing
p2brBack.range == p2br.range

######### What Distributions are supported ##############
using DataFrames
using CSV

distsToSupport = subtypes(Distributions.Distribution)
distNames = map(x -> string(x), distsToSupport)
distProtoTypesExist = Vector{Bool}(length(distNames))
distEncoderDecodersWork = Vector{Bool}(length(distNames))
index = 1
for dist in distNames
    info("Checking $dist...")
    try
        # If this passes we have a packed type
        packedName = replace(string(dist), "Distributions.", "Protos.")
        packedType = eval(parse(packedName))
        distProtoTypesExist[index] = true
    catch ex
        distProtoTypesExist[index] = false
    end

    try
        # Making one with default constructor.
        newDist = eval(parse(string(dist*"()")))
        # Now encode and decode to see if they work.
        distEncoderDecodersWork[index] = protoUnpackDistribution(protoPackDistribution(newDist)) == newDist
    catch ex
        # showerror(STDERR, ex, catch_backtrace())
        distEncoderDecodersWork[index] = false
    end

    index += 1
end

dfProtoCompatibilityTable = DataFrame(Distribution=distNames, ProtoTypesExist=distProtoTypesExist, EncoderDecodersWork=distEncoderDecodersWork)
CSV.write(joinpath(Pkg.dir("RoME"), "protos", "protodistributioncompat.csv"), dfProtoCompatibilityTable)

######### What Factors are supported ##############

factorsToSupport = [subtypes(IncrementalInference.FunctorSingleton);subtypes(IncrementalInference.FunctorPairwise);subtypes(IncrementalInference.FunctorPairwiseMinimize)]
factNames = map(x -> string(x), factorsToSupport)
factProtoTypesExist = Vector{Bool}(length(factNames))
factEncoderDecodersWork = Vector{Bool}(length(factNames))
index = 1
for fact in factNames
    info("Checking $fact...")
    try
        # If this passes we have a packed type
        packedName = replace(string(fact), "RoME.", "Protos.Packed")
        packedName = replace(packedName, "IncrementalInference.", "Protos.Packed")
        packedType = eval(parse(packedName))
        factProtoTypesExist[index] = true
    catch ex
        # showerror(STDERR, ex, catch_backtrace())
        factProtoTypesExist[index] = false
    end

    # try
    #     # Making one with default constructor.
    #     newDist = eval(parse(string(dist*"()")))
    #     # Now encode and decode to see if they work.
    #     distEncoderDecodersWork[index] = protoUnpackDistribution(protoPackFactor(newDist)) == newDist
    # catch ex
    #     # showerror(STDERR, ex, catch_backtrace())
        factEncoderDecodersWork[index] = false
    # end

    index += 1
end

dfProtoCompatibilityTable = DataFrame(Factor=factNames, FactorTypesExist=factProtoTypesExist, EncoderDecodersWork=factEncoderDecodersWork)
CSV.write(joinpath(Pkg.dir("RoME"), "protos", "protofactorcompat.csv"), dfProtoCompatibilityTable)
