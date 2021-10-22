# Factor Graph as a Whole

## Saving and Loading

Assuming some factor graph object has been constructed by hand or automation, it is often very useful to be able to store that factor graph to file for later loading, solving, analysis etc.  Caesar.jl provides such functionality through easy saving and loading.  To save a factor graph, simply do:
```julia
saveDFG("/somewhere/myfg", fg)
```

```@docs
saveDFG
```

Similarly in the same or a new Julia context, you can load a factor graph object
```julia
# using Caesar
fg_ = loadDFG("/somwhere/myfg")
```

```@docs
loadDFG
loadDFG!
```

!!! note
    Julia natively provides a direct in memory `deepcopy` function for making duplicate objects if you wish to keep a backup of the factor graph, e.g.
    ```julia
    fg2 = deepcopy(fg)
    ```

### Adding an `Entry=>Data` Blob store

A later part of the documentation will show [how to include a `Entry=>Data` blob store](https://juliarobotics.org/Caesar.jl/latest/concepts/entry_data/).

## Querying the FactorGraph

### List Variables:

A quick summary of the variables in the factor graph can be retrieved with:

```julia
# List variables
ls(fg)
# List factors attached to x0
ls(fg, :x0)
# TODO: Provide an overview of getVal, getVert, getBW, getBelief, etc.
```

It is possible to filter the listing with `Regex` string:
```julia
ls(fg, r"x\d")
```

```@docs
ls
```

```julia
unsorted = intersect(ls(fg, r"x"), ls(fg, Pose2))  # by regex

# sorting in most natural way (as defined by DFG)
sorted = sortDFG(unsorted)
```

```@docs
sortDFG
```

### List Factors:

```julia
unsorted = lsf(fg)
unsorted = ls(fg, Pose2Point2BearingRange)
```

or using the `tags` (works for variables too):
```julia
lsf(fg, tags=[:APRILTAGS;])
```

```@docs
lsf
lsfPriors
```

There are a variety of functions to query the factor graph, please refer to [Function Reference](../func_ref.md) for details and note that many functions still need to be added to this documentation.


### Extracting a Subgraph

Sometimes it is useful to make a deepcopy of a segment of the factor graph for some purpose:
```julia
sfg = buildSubgraph(fg, [:x1;:x2;:l7], 1)
```

# Extracting Belief Results (and PPE)

Once you have solved the graph, you can review the full marginal with:

```julia
X0 = getBelief(fg, :x0)
# Evaluate the marginal density function just for fun at [0.0, 0, 0].
X0(zeros(3,1))
```

This object is currently a Kernel Density which contains kernels at specific points on the associated manifold.  These kernel locations can be retrieved with:
```julia
X0pts = getPoints(X0)
```

```@docs
getBelief
```

## Parametric Point Estimates (PPE)

Since Caesar.jl is build around the each variable state being estimated as a total marginal posterior belief, it is often useful to get the equivalent parametric point estimate from the belief.  Many of these computations are already done by the inference library and avalable via the various [`getPPE`](@ref) methods, e.g.:
```julia
getPPE(fg, :l3)
getPPESuggested(fg, :l5)
```

There are values for mean, max, or hybrid combinations.

```@docs
getPPE
calcPPE
```

## Getting Many Marginal Samples

It is also possible to sample the above belief objects for more samples:
```julia
pts = rand(X0, 200)
```

## Building On-Manifold KDEs

These kernel density belief objects can be constructed from points as follows:
```julia
X0_ = manikde!(pts, Pose2)
```

## Logging Output (Unique Folder)

Each new factor graph is designated a unique folder in `/tmp/caesar`.  This is usaully used for debugging or large scale test analysis.  Sometimes it may be useful for the user to also use this temporary location.  The location is stored in the `SolverParams`:
```julia
getSolverParams(fg).logpath
```

The functions of interest are:
```@docs
getLogPath
joinLogPath
```

!!! note
    A useful tip for doing large scale processing might be to reduce amount of write operations to a solid-state drive that will be written to default location `/tmp/caesar` by simplying adding a symbolic link to a USB drive or SDCard, perhaps similar to:
    ```bash
    cd /tmp
    mkdir -p /media/MYFLASHDRIVE/caesar
    ln -s /media/MYFLASHDRIVE/caesar caesar
    ```

## Other Useful Functions

```@docs
getFactorDim
getManifolds
```