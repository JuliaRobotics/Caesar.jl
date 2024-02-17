# large dict using some store to hold values for reducing RAM utilization


using DataStructures
using UUIDs
using DocStringExtensions
using Serialization

import Base: getindex, setindex!, delete!, keys

##

"""
    $TYPEDEF

Walks and talks like a Dict but actually maps most of the data volume to a folder.
Includes semi-intelligent cache management for faster operations.

Special Features:
- User can set cache_size
- User can set working folder for storage
- User can set serialization and deserialization functions, e.g. use JSON3 or Serialization
- User can set how Dict keys map to stored id's (see DFG)
- EXPERIMENTAL: thread safe

Developer Notes
- all keys must always be in `.keydict`, regardless of cache or priority

WIP Constraints:
- Had trouble inheriting from `Base.AbstractDict`
"""
@kwdef struct FolderDict{K,V}
  """ regular dict elements kept in memory for rapid access """
  cache::Dict{K,V} = Dict{K,V}()
  """ priority queue for managing most important cache elements """
  pqueue::PriorityQueue{K, Int} = PriorityQueue{K,Int}()
  """ cache size """
  cache_size::Int = 100
  """ unique labels for dict elements sent to the store """
  keydict::Dict{K, UUID} = Dict{K, UUID}()
  """ mapping keys and ids for different use cases, default is always new uuid.
  overwrite with `(k) -> k` to make keys and ids identical """
  key_to_id::Function = (k) -> uuid4()
  """ read lock via Tasks """
  readtasks::Dict{K, Task} = Dict{K, Task}()
  """ write lock via Tasks """ 
  writetasks::Dict{K, Task} = Dict{K, Task}()
  """ working directory where elemental files are stored """
  wdir::String = begin
    wdir_ = joinpath(tempdir(), "$(uuid4())")
    mkpath(wdir_)
    wdir_
  end
  """ serialization function with default """
  serialize::Function = Serialization.serialize
  """ deserialization function with default """
  deserialize::Function = Serialization.deserialize
end

##


function Base.getindex(
  sd::FolderDict,
  f
)
  # first check if there is an ongoing writetask on this key
  if haskey(sd.writetasks, f)
    wait(sd.writetasks[f])
    # now it is safe to proceed in reading newly written value to this key
    # TODO slightly excessive lock, since only unlocks once storage write is done, but cache was available sooner.
  end
  # also check if there is an ongoing reader on this key
  if haskey(sd.readtasks, f)
    # NOTE super remote possibility that a task is deleted before this dict lookup and wait starts
    wait(sd.readtasks[f])
    # values should now be cached for multithreaded reads
  end
  
  # if already cached, and assuming a write task has not deleted the cache element yet (MUST delete from pqueue first)
  if haskey(sd.pqueue, f)
    # increase this priority, but be wary of over emphasis for newcomer keys
    sd.pqueue[f] += 1
    # Assume key must be in the cache
    return sd.cache[f]
  end
  # get id associated with this key (also throw KeyError if not present)
  flb = sd.keydict[f]
  # performance trick, start async load from IO resources while doing some housekeeping
  sd.readtasks[f] = @async begin
    # All keys must always be present in keydict 
    toload = joinpath(sd.wdir, "$flb")
    # fetch from cold storage
    sd.deserialize(toload)
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
  sd.pqueue[f] = round(Int, maxpriority/2) # pqueue is arbitor, hence populated last
  # block on IO resources fetching data
  # add to previously missed data to cache and pqueue
  sd.cache[f] = fetch(sd.readtasks[f])
  # TODO, possible race condition in slight delay betweem writing to cache[f] after fetching data unblocks.
  delete!(sd.readtasks, f)

  # return data to user
  return sd.cache[f]
end


function setindex!(
  sd::FolderDict,
  v,
  k
)
  # first check if there is an ongoing reader on this key
  if haskey(sd.readtasks, k)
    # NOTE super remote possibility that a task is deleted before this dict lookup and wait starts
    wait(sd.readtasks[k])
  end
  # also check if there is an ongoing writetask on this key
  if haskey(sd.writetasks, k)
    wait(sd.writetasks[k])
    # now it is safe to proceed in reading newly written value to this key
    # TODO slightly excessive lock, since only unlocks once storage write is done, but cache was available sooner.
  end

  # immediately/always insert new data into folder store with a unique id
  id = sd.key_to_id(k)
  flb = joinpath(sd.wdir, "$id")
  sd.writetasks[k] = @async sd.serialize(flb, v) # for sluggish IO
  
  # should any obsolete files be deleted from the filesystem?
  dtsk = if haskey(sd.keydict, k)
    # delete this store location
    dlb = sd.keydict[k] 
    delete!(sd.keydict, k)
    @async Base.Filesystem.rm(joinpath(sd.wdir, "$dlb")) # for sluggish IO
  else
    # dummy task for type stability
    @async nothing
  end
  # set new uuid in keydict only after potential overwrite delete
  sd.keydict[k] = id
  
  # ensure pqueue has correct value in all cases
  prk = collect(keys(sd.pqueue))
  maxpriority = 0
  # truncate for cache_size, and also allow for cache_size==0
  if 0 < sd.cache_size <= length(sd.pqueue)
    maxpriority = sd.pqueue[prk[end]]
    rmk = prk[1]
    delete!(sd.pqueue,rmk)
    delete!(sd.cache, rmk)
  end
  # assume middle of the pack priority for this cache-miss
  sd.pqueue[k] = round(Int, maxpriority/2)
  # Reminder, pqueue is arbitor over cache
  sd.cache[k] = v

  # wait for any disk mutations to finish
  wait(dtsk)
  # last thing is to wait and free write task locks, assuming waiting readers
  wait(sd.writetasks[k])
  delete!(sd.writetasks, k)

  # return the value
  return v
end


# # if multiple access occurs (i.e. shared memory)
# if haskey(sd.writetask, f)
#   # wait until the underlying task is complete
#   wait(sd.writetask[f]) # COUNTER MUST USE FETCH
# end


function delete!(
  sd::FolderDict,
  k
)
  dlb = sd.keydict[k] 
  delete!(sd.keydict, k)
  dtsk = @async Base.Filesystem.rm(joinpath(sd.wdir, "$dlb")) # for sluggish IO
  
  if haskey(sd.pqueue, k)
    delete!(sd.pqueue,k)
    delete!(sd.cache, k)
  end

  wait(dtsk)

  # TBD unusual Julia return of full collection
  return sd
end


keys(sd::FolderDict) = keys(sd.keydict)

##
