# a basic create robot type node example

using IncrementalInference, CloudGraphs, Caesar

# switch IncrementalInference to use CloudGraphs (Neo4j) data layer
# Caesar.useCloudGraphsDataLayer()
    # connect to the server, CloudGraph stuff
    configuration = CloudGraphs.CloudGraphConfiguration("localhost", 7474, false, "", "", "localhost", 27017, false, "", "");
    cloudGraph = connect(configuration);

    # register types of interest in CloudGraphs
    CloudGraphs.registerPackedType!(cloudGraph, VariableNodeData, PackedVariableNodeData, encodingConverter=VNDencoder, decodingConverter=VNDdecoder);
    CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{Obsv2}, FunctionNodeData{PackedObsv2}, encodingConverter=FNDencode, decodingConverter=FNDdecode)
    CloudGraphs.registerPackedType!(cloudGraph, FunctionNodeData{Odo}, FunctionNodeData{PackedOdo}, encodingConverter=FNDencode, decodingConverter=FNDdecode)


function addCloudVert!(fgl::FactorGraph, exvert::Graphs.ExVertex)
  cv = CloudGraphs.exVertex2CloudVertex(exvert);
  CloudGraphs.add_vertex!(cloudGraph, cv);
  fgl.cgIDs[exvert.index] = cv.neo4jNodeId
  IncrementalInference.addGraphsVert!(fgl, exvert)
end

iiapi = getCurrentAPI()
cgapi = DataLayerAPI(addCloudVert!,   # addvertex
                    iiapi.getvertex,   # getvertex
                    iiapi.setupvertgraph!,   # setupvertgraph
                    iiapi.setupfncvertgraph!,   # setupfncvertgraph
                    iiapi.makeedge,   # makeedge
                    iiapi.addedge!,   # addedge
                    iiapi.outneighbors,   # outneighbors
                    +, +, +, + )
IncrementalInference.setDataLayerAPI(cgapi)

# this is being replaced by cloudGraph, added here for development period
fg = emptyFactorGraph()
fg.cg = cloudGraph

# Robot navigation and inference type stuff
N=100
doors = [-100.0;0.0;100.0;300.0]'
cov = [3.0]

# robot style, add first pose vertex
v1 = addNode!(fg,"x1",doors,N=N)
# cv1 = CloudGraphs.exVertex2CloudVertex(v1);
# CloudGraphs.add_vertex!(cloudGraph, cv1);

f0  = addFactor!(fg,[v1], Obsv2(doors, cov', [1.0]))
# cv0 = CloudGraphs.exVertex2CloudVertex(f0);
# CloudGraphs.add_vertex!(cloudGraph, cv0);
ceA0 = CloudGraphs.CloudEdge(cv1, cv0, "DEPENDENCE");
CloudGraphs.add_edge!(cloudGraph, ceA0);

# add second pose vertex
tem = 2.0*randn(1,N)+getVal(v1)+50.0
v2 = addNode!(fg, "x2", tem, N=N)
cv2 = CloudGraphs.exVertex2CloudVertex(v2);
CloudGraphs.add_vertex!(cloudGraph, cv2);

# now add the odometry factor between them

f1 = addFactor!(fg,[v1;v2],Odo([50.0]',[2.0]',[1.0]))
cvA = CloudGraphs.exVertex2CloudVertex(f1);
CloudGraphs.add_vertex!(cloudGraph, cvA);
ceA1 = CloudGraphs.CloudEdge(cv1, cvA, "DEPENDENCE");
ceA2 = CloudGraphs.CloudEdge(cv2, cvA, "DEPENDENCE");
CloudGraphs.add_edge!(cloudGraph, ceA1);
CloudGraphs.add_edge!(cloudGraph, ceA2);

# get vertex back from DB
# cv1r = CloudGraphs.get_vertex(cloudGraph, 245, false)

# Get neighbors
neighs = CloudGraphs.get_neighbors(cloudGraph, cv1r)
