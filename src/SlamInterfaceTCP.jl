using Caesar
using IncrementalInference
using RoME
using CloudGraphs

@show dbaddress = ARGS[1]
configuration = CloudGraphs.CloudGraphConfiguration(dbaddress, 7474, "", "", dbaddress, 27017, false, "", "");
cloudGraph = connect(configuration);

# register types of interest in CloudGraphs
registerGeneralVariableTypes!(cloudGraph)
IncrementalInference.setCloudDataLayerAPI!()

nprocs() > 1 ? thxl = 2 : nothing

type SLAMWrapper
  fg::FactorGraph
  tree
  lndmidx::Int
end

USEREADY=0
include("SlamConvenienceFunctions.jl")


function tcpStringSLAMServer(;slamdata=Union{},port::Int=60001)
  println("Empty slam object created")
  if slamdata == Union{}
    # this is being replaced by cloudGraph, added here for development period
    fg = emptyFactorGraph()
    fg.cg = cloudGraph
    slamdata = SLAMWrapper(fg, Union{}, 0)
  end

  println("Listenting on $(port)")
  server = listen(port)
  loop = true
  while loop
   sock = accept(server)
     while isopen(sock)
         loop, retstr = parseTCP!(slamdata, readline(sock)[1:(end-1)])
         loop ? (isopen(sock) ? println(sock, retstr) : nothing) : close(sock)
     end
     println("connection lost")
  end
  !loop ? close(server) : nothing
  return slamdata
end

tcpStringSLAMServer()
