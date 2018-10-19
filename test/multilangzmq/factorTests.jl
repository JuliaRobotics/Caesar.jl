using Caesar, Caesar.ZmqCaesar

# AliasingScalarSampler
packed = Packed_AliasingScalarSampler(collect(1:10), collect(11:20), 0, "AliasingScalarSampler")
json = JSON.json(packed)
sampler = convert(IncrementalInference.AliasingScalarSampler, JSON.parse(json))
packedCompare = convert(Packed_AliasingScalarSampler, sampler)
jsonCompare = JSON.json(packedCompare)
