using ProtoBuf
using ProtoBuf.Gen
run(ProtoBuf.protoc(`-I=proto --julia_out=jlout distributions/normal.proto`))
