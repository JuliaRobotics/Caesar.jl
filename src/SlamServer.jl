

# look at SimpleExample.jl for TCP based usage example

nprocs() > 1 ? thxl = 2 : nothing


type SLAMWrapper
  fg::FactorGraph
  tree
  lndmidx::Int # TODO -- move this to FactorGraph type
end

function prepString(arr::Array{Float64,2})
  s = string(arr)
  ss = split(s,'\n')
  ss[length(ss)] = ss[length(ss)][1:(end-1)]
  fs = ""
  for i in 1:length(ss)
    ss[i]=replace(ss[i]," ",",")
    if ss[i][1] == ',' || ss[i][1] == '['
      ss[i] = ss[i][2:end]
    end
    ss[i] = string(ss[i],";")
    fs = string(fs,ss[i])
  end
  fs[1:(end-1)]
end


function parseInit!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  if length(sp2) == 3
    println("parseInit! -- received initialization point $(sp2) NOT CURRENTLY USED")
    x0 = parse(Float64,sp2[1])
    y0 = parse(Float64,sp2[2])
    th0 = parse(Float64,sp2[2])
  end
  prevn = initFactorGraph!(slam.fg)
  println("init done")
  nothing
end

posecount = 1
function parseOdo!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  global posecount
  # println("parseOdo -- ")
  posecount += 1
  n = string("x", posecount)
  DX = [parse(Float64,sp2[3]);parse(Float64,sp2[4]);parse(Float64,sp2[5])]
  cov = diagm([parse(Float64,sp2[6]);parse(Float64,sp2[9]);parse(Float64,sp2[11])])
  cov[1,2] = parse(Float64,sp2[7])
  cov[1,3] = parse(Float64,sp2[8])
  cov[2,3] = parse(Float64,sp2[10])
  cov[2,1] = parse(Float64,sp2[7])
  cov[3,1] = parse(Float64,sp2[8])
  cov[3,2] = parse(Float64,sp2[10])
  addOdoFG!(slam.fg, n, DX, cov)
  return n
end

function parseAddLandmBR!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  poseid = floor(Int,parse(Float64, sp2[1]))
  pose = ASCIIString(slam.fg.v[poseid].label)
  lmid = floor(Int,parse(Float64, sp2[2]))
  zbr = [parse(Float64,sp2[3]);parse(Float64,sp2[4])]
  cov = diagm([parse(Float64,sp2[5]);parse(Float64,sp2[7])])
  cov[1,2] = parse(Float64,sp2[6])
  cov[2,1] = parse(Float64,sp2[6])
  lm = ASCIIString("")
  if !haskey(slam.fg.v, lmid)
    slam.lndmidx += 1
    lm = ASCIIString(string('l',slam.lndmidx))
    projNewLandm!(slam.fg, pose, lm, zbr, cov)
  else
    lm = ASCIIString(slam.fg.v[lmid].label)
    addBRFG!(slam.fg, pose, lm, zbr, cov)
  end
  return lm
end

function parseLandmBRMM!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  poseid = floor(Int,parse(Float64, sp2[1]))
  pose = ASCIIString(slam.fg.v[poseid].label)
  lm1id = floor(Int,parse(Float64, sp2[2]))
  w1 = parse(Float64, sp2[3])
  lm2id = floor(Int,parse(Float64, sp2[4]))
  w2 = parse(Float64, sp2[5])
  zbr = [parse(Float64,sp2[6]);parse(Float64,sp2[7])]
  cov = diagm([parse(Float64,sp2[8]);parse(Float64,sp2[10])])
  cov[1,2] = parse(Float64,sp2[9])
  cov[2,1] = parse(Float64,sp2[9])
  # TODO --do the tests here
  lm1 = ASCIIString(slam.fg.v[lm1id].label)
  lm2 = ASCIIString(slam.fg.v[lm2id].label)
  addMMBRFG!(slam.fg, pose, [lm1;lm2], zbr, cov, w=[w1;w2])
  # function addMMBRFG!(fg::FactorGraph, pose::ASCIIString,
  #                   lm::Array{ASCIIString,1}, br::Array{Float64,1},
  #                   cov::Array{Float64,2}; w=[0.5;0.5])
  return "$(lm1), $(lm2)"
end

function parseLandmBRAuto!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  poseid = floor(Int,parse(Float64, sp2[1]))
  pose = ASCIIString(slam.fg.v[poseid].label)
  zbr = [parse(Float64,sp2[2]);parse(Float64,sp2[3])]
  cov = diagm([parse(Float64,sp2[4]);parse(Float64,sp2[6])])
  cov[1,2] = parse(Float64,sp2[5])
  cov[2,1] = parse(Float64,sp2[5])
  lm = ASCIIString("")

  vlm, flm = addAutoLandmBR!(slam.fg, pose, zbr, cov)

  println("parseLandmBRAuto! -- added $(vlm.label)")

  return vlm.label
end


function parseLandmarkXY!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  pose = slam.fg.v[map(Int, sp2[1])].label
  lmid = map(Int, sp2[2])
  zxy = [map(Float64,sp[3]);map(Float64,sp[4])]
  cov = diagm([map(Float64,sp[5]);map(Float64,sp[7])])
  cov[1,2] = map(Float64,sp[6])
  cov[2,1] = map(Float64,sp[6])
  error("parseLandmarkXY! -- not finished implementing yet")
  nothing
end


function batchSolve!(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  println("batchSolve -- wiping tree and solving")
  slam.tree = wipeBuildNewTree!(slam.fg)
  @time inferOverTree!(slam.fg, slam.tree, N=100)
  nothing
end

function parseGetparticles(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  id = slam.fg.IDs[sp2[1]]
  val = getVal(slam.fg.v[id])
  retstr = prepString(val)
  return retstr
end

function parseLS(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  xx,ll = ls(slam.fg)
  str = ASCIIString("")
  [str = string(str, x, ",") for x in xx]
  if length(str) > 0
    str = string(str[1:(end-1)], ';')
  end
  [str = string(str, x, ",") for x in ll]
  @show "LS", str
  return length(str) > 0 ? str : nothing
end

function parseGetID(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  id = slam.fg.IDs[sp2[1]]
  return "$(id)"
end

function parseReset(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  warn("parseReset -- resetting factor graph")
  slam = SLAMWrapper(emptyFactorGraph(), Union{}, 0)
  nothing
end

function parseDotFG(slam::SLAMWrapper, sp2::Array{SubString{ASCIIString},1})
  writeGraphPdf(slam.fg)
  nothing
end

function parseTCP!(slam::SLAMWrapper, line::ASCIIString)
    sp = split(line,' ')
    f = +
    cmd = sp[1]
    goahead = true
    if cmd == "INIT"
      f = parseInit!
    elseif cmd == "ODOMETRY"
      f = parseOdo!
    elseif cmd == "LANDMBR"
      f = parseAddLandmBR!
    elseif cmd == "LANDMARK"
      f = parseLandmarkXY!
    elseif cmd == "LANDMBRMM"
      f = parseLandmBRMM!
    elseif cmd == "LANDMBRAUTO"
      f = parseLandmBRAuto!
    elseif cmd == "BATCHSOLVE"
      f = batchSolve!
    elseif cmd == "GETPARTICLES"
      f = parseGetparticles
    elseif cmd == "LS"
      f = parseLS
    elseif cmd == "GETID"
      f = parseGetID
    elseif cmd == "DRAWFGPDF"
      f = parseDotFG
    elseif cmd == "RESET"
      f = parseReset
    elseif cmd == "QUIT"
      println("parseTCP -- should quit now")
      return false, ASCIIString("")
    else
      warn("parseTCP -- I don't know what $(cmd) means")
      goahead = false
    end
    retstr = nothing
    goahead ? retstr = f(slam, sp[2:end]) : nothing
    return true, retstr
end


function tcpStringSLAMServer(;port::Int=60001)
  println("Empty slam object created")
  slam001 = SLAMWrapper(emptyFactorGraph(), Union{}, 0)

  println("Listenting on $(port)")
  server = listen(port)
  loop = true
  while loop
   sock = accept(server)
     while isopen(sock)
         loop, retstr = parseTCP!(slam001, readline(sock)[1:(end-1)])
         loop ? (isopen(sock) ? println(sock, retstr) : nothing) : close(sock)
     end
     println("connection lost")
  end
  !loop ? close(server) : nothing
  nothing
end
