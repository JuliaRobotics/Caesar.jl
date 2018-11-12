using Caesar, Caesar.ZmqCaesar
using Distributions

# Distribution: AliasingScalarSampler
packed = Packed_AliasingScalarSampler(collect(1:10), collect(11:20), 0, "AliasingScalarSampler")
json = JSON.json(packed)
sampler = convert(IncrementalInference.AliasingScalarSampler, JSON.parse(json))
packedCompare = convert(Packed_AliasingScalarSampler, sampler)
jsonCompare = JSON.json(packedCompare)

# Distribution: Normal
normal = Normal(5.0, 10.0)
packed = convert(Dict{String, Any}, normal)
back = convert(Normal, JSON.parse(JSON.json(packed)))

# Factor: PartialPriorRollPitchZ
a = MvNormal([0, 0, 0, 0], eye(4))
b = Normal(0, 1)
partPrior = PartialPriorRollPitchZ(a, b)
j = JSON.json(convert(Dict{String, Any}, partPrior))
# Test the deserializer
back = convert(PartialPriorRollPitchZ, JSON.parse(j))
jback = JSON.json(convert(Dict{String, Any}, back))
jback = j
