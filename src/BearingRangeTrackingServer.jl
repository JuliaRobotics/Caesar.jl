
# BearingRangeTrackingServer

function parseProcTracking!(instSys::InSituSystem, d::Dict{T, Any}) where {T <: AbstractString}
    # parameters
    lsrnoise = []

    # unpack the data
    b1Dxb = Array{Float64,1}(d["dx"])
    lsrFeats = d["BRfeats"]
    len = length(lsrFeats)
    bfts = zeros(3,len)
    for i in 1:len, j in 1:3
      bfts[j,i] = Float64(lsrFeats[i][j])
    end
    # do the work
    propAllTrackers!(instSys.trackers, b1Dxb, [0.05;0.05;0.004])
    hardassc = assocMeasWFeats!(instSys.trackers, bfts)
    measUpdateTrackers!(instSys.trackers, hardassc, [0.4;0.02])

    # pack outgoing data
    od = Dict{Int64,Array{Float64,1}}()
    for t in instSys.trackers
      m = getKDEMean(t[2].bel) #getKDEMax
      od[t[1]] = cart2pol(m, [1.0;1.0])[1]
      # println("tracker $(t[1]), pol=$(od[t[1]])")
    end
    println("tracking $(length(instSys.trackers)) feats")
    rmsg = json(od)
    return rmsg # json(current tracked feature jsoned dict)
end


function parseTCP!(insitu::InSituSystem, line::AbstractString)
    # sp = split(line,' ')
    # cmd = sp[1]

    f = +
    goahead = true
    d = Dict{AbstractString, Any}()
    try
      d = JSON.parse(line)
      cmd = AbstractString(d["CMD"])
      # if cmd == "INIT"
      #   f = parseInit!
      if cmd == "PROCESS"
        f = parseProcTracking!
      # elseif cmd == "RESET"
      #   f = parseReset
      elseif cmd == "QUIT"
        println("parseTCP -- should quit now")
        return false, string("")
      else
        warn("parseTCP -- I don't know what $(cmd) means")
        goahead = false
      end
    catch
      warn("parseTCP! -- json parse and command failed: $(line)")
      goahead = false
    end
    retstr = nothing
    goahead ? retstr = f(insitu, d) : nothing
    return true, retstr
end

# function propAllTrackers!(trkrs::Dict{Int64,Feature}, bDxb1::Array{Float64,1}, s::Array{Float64,1})


function tcpStringBRTrackingServer(;port::Int=60002)
  println("Empty teacker object created")
  instSys = makeInSituSys(zeros(3), zeros(2,0))

  println("Listenting on $(port)")
  server = listen(port)
  loop = true
  while loop
   sock = accept(server)
     while isopen(sock)
         gotline = readline(sock)[1:(end-1)]
         loop, retstr = parseTCP!(instSys, gotline)
         loop ? ( isopen(sock) ? println(sock, retstr) : nothing ) : close(sock)
     end
     println("connection lost, loop=$(loop)")
  end
  !loop ? close(server) : nothing
  return instSys
end
