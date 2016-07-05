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

# add second pose vertex
tem = 2.0*randn(1,N)+getVal(v1)+50.0
v2 = addNode!(fg, "x2", tem, N=N)
cv2 = CloudGraphs.exVertex2CloudVertex(v2);
CloudGraphs.add_vertex!(cloudGraph, cv2);

# now add the odometry factor between them
# but first we need to register the new types we will be using
CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{Odo}, FunctionNodeData{PackedOdo}, encodingConverter=FNDencode, decodingConverter=FNDencode)

f1 = addFactor!(fg,[v1;v2],Odo([50.0]',[2.0]',[1.0]))
cvA = CloudGraphs.exVertex2CloudVertex(f1);
CloudGraphs.add_vertex!(cloudGraph, cvA);
ceA1 = CloudGraphs.CloudEdge(cv1, cvA, "DEPENDENCE");
ceA2 = CloudGraphs.CloudEdge(cv2, cvA, "DEPENDENCE");
CloudGraphs.add_edge!(cloudGraph, ceA1);
CloudGraphs.add_edge!(cloudGraph, ceA2);
