# large dict using some store to hold values for reducing RAM utilization


using DataStructures
using UUIDs
using Serialization

import Base: getindex, setindex!

##


# not yet thread safe
# all keys must always be in keydict, regardless of cache or priority
@kwdef struct FolderDict{K,V}
  # regular dict elements kept in memory for rapid access
  cache::Dict{K,V} = Dict{K,V}()
  # priority queue for managing most important cache elements
  pqueue::PriorityQueue{K, Int} = PriorityQueue{K,Int}()
  # cache size
  cache_size::Int = 100
  # unique labels for dict elements sent to the store
  keydict::Dict{K, UUID} = Dict{K, UUID}()
  # # write lock via Task to 
  # writetask::Dict{K, Task} = Dict{K, Task}()
  wdir::String = begin
    wdir_ = joinpath(tempdir(), "$(uuid4())")
    mkpath(wdir_)
    wdir_
  end
end

##


function Base.getindex(
  sd::FolderDict,
  f
)
  # if already cached, and assuming a write task has not deleted the cache element yet (MUST delete from pqueue first)
  if haskey(sd.pqueue, f)
    # increase this priority, but be wary of over emphasis for newcomer keys
    sd.pqueue[f] += 1
    # Assume key must be in the cache
    return sd.cache[f]
  end
  # performance trick, start async load from IO resources while doing some housekeeping
  tsk = @async begin
    # All keys must always be present in keydict 
    flb = sd.keydict[f]
    toload = joinpath(sd.wdir, "$flb")
    # fetch from cold storage
    Serialization.deserialize(toload)
  end

  # you've hit a cache miss, so start making space or new elements
  _typemin(::PriorityQueue{P,T}) where {P,T} = typemin(T)
  maxpriority = _typemin(sd.pqueue)
  for k in keys(sd.pqueue)
    # decrement all priorities by one - to help newcomer keys compete for priority
    sd.pqueue[k] -= 1
    # preemptively find the maxpriority value for later use
    maxpriority = maxpriority < sd.pqueue[k] ? sd.pqueue[k] : maxpriority
  end

  # remove excess cache if necessary 
  if sd.cache_size <= length(sd.pqueue)
    # by dropping lowest priority cache element
    dropkey = first(sd.pqueue)[1]
    delete!(sd.pqueue, dropkey)
    delete!(sd.cache, dropkey)
  end

  # assume middle of the pack priority for this cache-miss
  newp = round(Int, maxpriority/2)
  # block on IO resources fetching data
  data = fetch(tsk)

  # add to previously missed data to cache and pqueue
  sd.cache[f] = data # cache first
  sd.pqueue[f] = newp # pqueue is arbitor, hence populated last

  # return data to user
  return data
end


function setindex!(
  sd::FolderDict,
  v,
  k
)
  # immediately/always insert new data into folder store with newly generated uuid
  nid = uuid4()
  flb = joinpath(sd.wdir, "$nid")
  wtsk = @async begin
    Serialization.serialize(flb, v) # for sluggish IO
    @info "DOSE2T" flb
  end
  
  # should any obsolete files be deleted from the filesystem?
  dtsk = if haskey(sd.keydict, k)
    # delete this store location
    dlb = sd.keydict[k] 
    delete!(sd.keydict, k)
    @async Base.Filesystem.rm(joinpath(sd.wdir, "$dlb")) # for sluggish IO
  end
  # set new uuid in keydict only after potential overwrite delete
  sd.keydict[k] = nid
  
  # should the cache be updated? pqueue is arbitor over cache
  if haskey(sd.pqueue, k)
    sd.cache[k] = v
    # maintain the old priority value
  end

  # write new info to FolderDict fields
  sd.keydict[k] = nid

  # wait for any disk mutations to finish
  _wait(::Nothing) = nothing
  _wait(s::Task) = wait(s)
  _wait(dtsk)
  wait(wtsk)

  # return the value
  return v
end


# # if multiple access occurs (i.e. shared memory)
# if haskey(sd.writetask, f)
#   # wait until the underlying task is complete
#   wait(sd.writetask[f]) # COUNTER MUST USE FETCH
# end



##
