# a basic create robot type node example

using IncrementalInference, CloudGraphs, Caesar

# switch IncrementalInference to use CloudGraphs (Neo4j) data layer
# Caesar.useCloudGraphsDataLayer()
    # connect to the server, CloudGraph stuff
    configuration = CloudGraphs.CloudGraphConfiguration("localhost", 7474, false, "", "", "localhost", 27017, false, "", "");
    cloudGraph = connect(configuration);
    # register types of interest in CloudGraphs
    CloudGraphs.registerPackedType!(cloudGraph, VariableNodeData, PackedVariableNodeData, encodingConverter=VNDencoder, decodingConverter=VNDdecoder);

# Robot navigation and inference type stuff
N=100
doors = [-100.0;0.0;100.0;300.0]'
cov = [3.0]
fg = emptyFactorGraph() # this will is being replaced by cloudGraph, added here for development period

# robot style, add first pose vertex
v1 = addNode!(fg,"x1",doors,N=N)

cv1 = CloudGraphs.exVertex2CloudVertex(v1);
CloudGraphs.add_vertex!(cloudGraph, cv1);
