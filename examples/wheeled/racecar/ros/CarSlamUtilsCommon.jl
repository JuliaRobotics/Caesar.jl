# common car camera tools

## clear old memory

delete!(vis[:tags])
delete!(vis[:poses])

##

abstract type SynchronizingBuffer end

struct SynchronizeCarMono <: SynchronizingBuffer
  syncList::Vector{Symbol}
  leftFwdCam::CircularBuffer{Tuple{Int, Any}}
  rightFwdCam::CircularBuffer{Tuple{Int, Any}}
  camOdo::CircularBuffer{Tuple{Int, Any}}
  cmdVal::CircularBuffer{Tuple{Int, Any}}
end

SynchronizeCarMono(len::Int=30;
                   syncList::Vector=Symbol[],
                   leftFwdCam=CircularBuffer{Tuple{Int, Any}}(len),
                   rightFwdCam=CircularBuffer{Tuple{Int, Any}}(len),
                   camOdo=CircularBuffer{Tuple{Int, Any}}(len),
                   cmdVal=CircularBuffer{Tuple{Int, Any}}(len) ) = SynchronizeCarMono(syncList,leftFwdCam,rightFwdCam,camOdo,cmdVal)
#

struct RacecarTools
  tagDetector
end

struct RacecarVisualizations
  gui
  vis
end

struct FrontEndContainer{M,T,A}
  slam::SLAMWrapperLocal
  middleware::M
  synchronizer::T
  tools::A
end

FrontEndContainer(s::SLAMWrapperLocal,m::M,t::T,a::A) where {M, T, A} = FrontEndContainer{M,T,A}(s,m,t,a)

## data management functions

# function getSyncLatestPair(syncHdlrs::Dict; weirdOffset::Dict=Dict())::Tuple
function findSyncLatestIdx(syncz::SynchronizingBuffer;
                           weirdOffset::Dict=Dict(),
                           syncList::Vector{Symbol}=syncz.syncList)
  # get all sequence numbers
  len = length(syncList)
  if len == 1
    # simply return the newest element in the buffer
    sy = getfield(syncz, syncList[1])
    return length(sy)
  elseif len == 0
    @warn "findSyncLatestIdx has nothing to synchronize -- on account of .syncList being empty."
    return 0  # error
  end
  LEN = Vector{Int}(undef, len)
  for idx in 1:len
    LEN[idx] = getfield(syncz, syncList[idx]) |> length
    if LEN[idx] == 0
      return (NTuple{len, Int}(zeros(len)))
    end
  end

  SEQ = Vector{Vector{Int}}(undef, len)
  for idx in 1:len
    SEQ[idx] = (x->getfield(syncz, syncList[idx])[x][1]).(1:LEN[idx])
    if haskey(weirdOffset, syncList[idx])
      SEQ[idx] .+= weirdOffset[syncList[idx]]
    end
  end

  # find greatest common number
  seqLR = intersect(SEQ...)
  if 0 == length(seqLR)
    return NTuple{len,Int}(zeros(len))
  end
  seq = maximum(seqLR)
  # IDX = Vector{Int}(undef, len)

  return NTuple{len, Int}( (y->findfirst(x->x==seq, y)).(SEQ) )
end




## Visualization functions

function showImage(image, tags, K)
    # Convert image to RGB
    imageCol = RGB.(image)
    # draw the tag number for each tag
    foreach(x->drawTagID!(imageCol, x),tags)
    #draw color box on tag corners
    foreach(tag->drawTagBox!(imageCol, tag, width = 2, drawReticle = false), tags)
    foreach(tag->drawTagAxes!(imageCol,tag, K), tags)
    imageCol
end


#
