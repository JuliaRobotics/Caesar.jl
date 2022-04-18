# [EXPL Stash and Cache](@id section_stash_and_cache)

!!! warning
    Stashing and Caching are new EXPERIMENTAL features (22Q2) and is not yet be fully integrated throughout the overall system.  See [Notes](@ref stashcache_notes) below for specific considerations.

Caching aims to improve in-place, memory, and communication bandwidth requirements for factor calculations and serialization.

## Preamble Cache

The Caesar.jl framework has a standardized feature to preload or cache important data for factor calculations the first time a factor is created/loaded into a graph (i.e. during [`addFactor!`](@ref)).  The [`preambleCache`](@ref) function runs just once before any computations are performed.  A default dispatch for `preambleCache` returns `nothing` as a cache object that is later used in several places throughout the code.

### Overriding `preambleCache`

A user may choose to override the dispatch for a particular factor's `preambleCache` and thereby return a more intricate/optimized cache object for later use.  Any object can be returned, but we strongly recommend you return a type-stable object for best performance in production.  Returning non-concrete types is allowed and likely faster for development, just remember to check type-stability before calling it a day.

Whatever object is returned by the `preambleCache(dfg, vars, fnc)` function is referenced and duplicated within the solver code.  During the design, use of cache is expected to predominantly occur during factor sampling, factor residual calculations, and deserialization (i.e. unpacking) of previously persisted graph objects.

The `preambleCache` function has access to the parent factor graph object as well as an ordered list of the `DFGVariable`s attached to said factor.  The user created factor type objects passed as the third argument.  The combination of these three objects allows the user much freedom wrt to where and how large data might be stored in the system.

```@docs
preambleCache
```

### In-Place vs. In-Line Cache

Depending on your particular bent, two different cache models might be more appealing.  The design of `preambleCache` does not preclude either design options, and actually promote use of either depending on the particular situation at hand.  The purpose of `preambleCache` is to provide and opportunity for caching when working with factors in the factor graph rather than dictate one design over the other.

#### `CalcFactor.cache::T`

One likely use of the `preambleCache` function is for in-place memory allocation for solver hot-loop operations.  Consider for example a [`getSample`](@ref) or factor residual calculation that is memory intensive.  The best way to improve performance is remove any memory allocations during the hot-loop.  For this reason the `CalcFactor` object has a `cache::T` field which will have exactly the type `::T` that is returned by the user's `preambleCache` dispatch override.  To usein the factor `getSample` or residual functions, simply use the `calcfactor.cache` field.

#### Pulling Data from Stores

The Caesar.jl framework [supports various data store](@ref section_data_entry_blob_store) designs.  Some of these data stores are likely best suited for in-line caching design.  Values can be [retrieved from a data store](@ref section_retrieve_data_blob) during the `preambleCache` step, irrespective of where the data is stored.  

If the user chooses to store weird and wonderful caching links to alternative hardware via the described caching, go forth and be productive!  Consider sharing enhancements back the public repositories.

## Stash Serialization

!!! note
    Stashing uses [Additional (Large) Data](@ref section_data_entry_blob_store) storage and retrieval following starved graph design considerations.

Some applications use graph factors with with large memory requirements during computation.  Often, it is not efficient/performant to store large data blobs directly within the graph when persisted.  Caesar.jl therefore supports a concept called _stashing_ (similar to starved graphs), where particular operationally important data is stored separately from the graph which can then be retrieved during the `preambleCache` step -- a.k.a. _unstashing_.

### Deserialize-only Stash Design (i.e. unstashing)

Presently, we recommend following a deserialize-only design.  This is where factor graph are reconstituted from some persisted storage into computable form in memory, a.k.a. [`loadDFG`](@ref).  During the load steps, factors are added to the destination graph using the `addFactor!` calls, which in turn call `preambleCache` for each factor.  

Therefore,  factors which are persisted using the 'stash' methodology are only fully reconstructed after the `preambleCache` step, and the user is responsible for defining the `preambleCache` override for a particular factor.  The desired stashed data should also already be available in said data store before the factor graph is loaded.

Caesar.jl does have factors that can use the stash design, but are currently only available as experimental features.  Specifically, see the [`ScatterAlignPose2`](@ref) factor code.

Modifying the overall Caesar.jl code for both read and write stashing might be considered in future work but is not in the current roadmap.

## [Notes](@id stashcache_notes)

Please see or open issues for specific questions not yet covered here.  You can also reach out via [Slack](https://join.slack.com/t/caesarjl/shared_invite/zt-ucs06bwg-y2tEbddwX1vR18MASnOLsw), or contact [NavAbility.io](https://www.navability.io) for help.

- Use caution in designing [`preambleCache`](@ref) for situations where [`multihypo=`](@ref section_multihypo) functionality is used.  If factor memory is tied to specific variables, then the association ambiguities to multihypo situations at compute time must considered.  E.g. if you are storing images for two landmarks in two landmark variable hypotheses, then just remember that the user cache must during the sampling or residual calculations track which hypothesis is being used before using said data -- we recommend using `NamedTuple` in your cache structure.
- If using the Deserialize-Stash design, note that the appropriate data blob stores should already be attached to the destination factor graph object, else the `preambleCache` function will not be able to succesfully access any of the `getData` functions you are likely to use to 'unstash' data.
- Users can readily implement their own threading inside factor samping and residual computations.  Caching is not yet thread-safe for some internal solver side-by-side computations, User can self manage shared vs. separate memory for the `Multithreaded Factor` option, but we'd recommend reaching out to or getting involved with the threading redesign, see [IIF 1094](https://github.com/JuliaRobotics/IncrementalInference.jl/issues/1094).