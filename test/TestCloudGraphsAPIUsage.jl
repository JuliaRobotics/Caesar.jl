# test will live in Caesar.jl

using IncrementalInference, CloudGraphs


IncrementalInference.setAPI( DataLayerAPI(
CloudGraphs.add_vertex!,
CloudGraphs.get_vertex,
CloudGraphs.make_edge,
CloudGraphs.add_edge!,
CloudGraphs.out_neighbors,
CloudGraphs.update_vertex!
CloudGraphs.update_edge!
) )



# Example using CloudGraphs DB functionality
# Do fourdoorstest

# initCloudGrphMagicSomething()....



#CloudGraph.registerPackedType!(cloudGraph, IncrementalInference.FunctionNodeData{PackedOdo},
#                               coolencode, cooldecode)
