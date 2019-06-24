
# look at SimpleExample.jl for TCP based usage example

nprocs() > 1 ? thxl = 2 : nothing


USEREADY=1
include("SlamConvenienceFunctions.jl")


function tcpStringSLAMServer(;slamdata=nothing,port::Int=60001)
  println("Empty slam object created")
  if slamdata == nothing
    slamdata = SLAMWrapper(emptyFactorGraph(), nothing, 0)
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
