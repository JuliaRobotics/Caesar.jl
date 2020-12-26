# Principle: Bayes tree prototyping

This page describes how to visualize, study, test, and compare Bayes (Junction) tree concepts with special regard for variable ordering.

## Why a Bayes (Junction) tree

The tree is algebraicly equivalent---but acyclic---structure to the factor graph:  i.) Inference is easier on on acyclic graphs; ii.) We can exploit Smart Message Passing benefits (known from the full conditional independence structure encoded in the tree), since the tree represents the "complete form" when marginalizing each variable one at a time (also known as elimination game, marginalization, also related to smart factors).  In loose terms, the Bayes (Junction) tree has implicit access to all Schur complements (if it parametric and linearized) of each variable to all others.  Please see [this page more information regarding advanced topics on the Bayes tree](https://www.juliarobotics.org/Caesar.jl/latest/principles/initializingOnBayesTree/).

## What is a Bayes (Junction) tree

The Bayes tree data structure is a rooted and directed Junction tree (maximal elimination clique tree). It allows for exact inference to be carried out by leveraging and exposing the variables' conditional independence and, very interestingly, can be directly associated with the sparsity pattern exhibited by a system's factorized upper triangular square root information matrix (see picture below).

![graph and matrix analagos](https://user-images.githubusercontent.com/27132241/69210533-f55da400-0b52-11ea-89dd-f18b7fa983b8.png)

Following this matrix-graph parallel, the picture also shows what the associated matrix interpretation is for a factor graph (~first order expansion in the form of a measurement Jacobian) and its corresponding Markov random field (sparsity pattern corresponding to the information matrix).

The procedure for obtaining the Bayes (Junction) tree is outlined in the figure shown below (factor graph to chrodal Bayes net via bipartite elimination game, and chordal Bayes net to Bayes tree via maximum cardinality search algorithm).

![add the fg2net2tree outline](https://user-images.githubusercontent.com/27132241/69210647-5eddb280-0b53-11ea-82ab-dc5ff89c4a43.png)

## Constructing a Tree

!!! note
    A visual illustration of factor graph to Bayes net to Bayes tree can be [found in this PDF](https://github.com/JuliaRobotics/IncrementalInference.jl/files/3929194/hex-slam.pdf) 

Trees and factor graphs are separated in the implementation, allowing the user to construct multiple different trees from one factor graph except for a few temporary values in the factor graph.

```julia
using IncrementalInference # RoME or Caesar will work too

## construct a distributed factor graph object
fg = generateCanonicalFG_Kaess()
# add variables and factors
# ...

## build the tree
tree = resetBuildTree!(fg)
```

The temporary values are `reset` from the distributed factor graph object `fg<:AbstractDFG` and a new tree is constructed.  This `resetBuildTree!` call can be repeated as many times the user desires and results should be consistent for the same factor graph structure (regardless of numerical values contained within).

```@docs
resetBuildTree!
```

## Variable Ordering

### Getting the AMD Variable Ordering

The variable ordering is described as a `::Vector{Symbol}`.  Note the automated methods can be varied between AMD, CCOLAMD, and others.
```julia
# get the automated variable elimination order
vo = getEliminationOrder(fg)
```

It is also possible to manually define the Variable Ordering
```julia
vo = [:x1; :l3; :x2; ...]
```

And then reset the factor graph and build a new tree
```julia
resetBuildTreeFromOrder!(fg, vo)
```

```@docs
resetBuildTreeFromOrder!
```

!!! note
    a list of variables or factors can be obtained through the `ls` and related functions, see [Querying the FactorGraph](@ref).


## Interfacing with the MM-iSAMv2 Solver

The following parmaters (set before calling `solveTree!`) will show the solution progress on the tree visualization:
```julia
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true

# asybc process will now draw and show the tree in linux
tree, smt, hist = solveTree!(fg)
```

!!! note
    See the [Solving Graphs](@ref) section for more details on the solver.

### Get the Elimination Order Used

The solver internally uses [`resetBuildTree!`](@ref) which sometimes requires the user extract the variable elimination order after the fact.  This can be done with:
```@docs
getEliminationOrder
```

## Visualizing

IncrementalInference.jl includes functions for visualizing the Bayes tree, and uses outside packages such as GraphViz (standard) and Latex tools (experimental, optional) to do so.  

### GraphViz

```julia
drawTree(tree, show=true) # , filepath="/tmp/caesar/mytree.pdf"
```

```@docs
drawTree
```

### Latex Tikz (Optional)

**EXPERIMENTAL**, requiring special import.

First make sure the following packages are installed on your system:
```
$ sudo apt-get install texlive-pictures dot2tex
$ pip install dot2tex
```

Then in Julia you should be able to do:
```julia
import IncrementalInference: generateTexTree

generateTexTree(tree)
```

An example Bayes (Junction) tree representation obtained through `generateTexTree(tree)` for the sample factor graph shown above can be seen in the following image.

```@raw html
<p align="center">
<img src="https://user-images.githubusercontent.com/27132241/69210722-9e0c0380-0b53-11ea-9462-7964844b89b1.png" width="200" border="0" />
</p>
```

### Visualizing Clique Adjacency Matrix

It is also possible to see the upward message passing variable/factor association matrix for each clique, requiring the Gadfly.jl package:
```julia
using Gadfly

spyCliqMat(tree, :x1) # provided by IncrementalInference

#or embedded in graphviz
drawTree(tree, imgs=true, show=true)
```

## Clique State Machine

The mmisam solver is based on a state machine design to handle the inter and intra clique operations during a variety of situations.  Use of the clique state machine (CSM) makes debugging, development, verification, and modification of the algorithm real easy.  Contact us for any support regarding modifications to the default algorithm.  For pre-docs on working with CSM, please see [IIF #443](https://github.com/JuliaRobotics/IncrementalInference.jl/issues/443).

### STATUS of a Clique

CSM currently uses the following statusses for each of the cliques during the inference process.

```julia
[:initialized;:upsolved;:marginalized;:downsolved;:uprecycled]
```

### Bayes Tree Legend (from IIF)

The color legend for the refactored CSM from [issue](https://github.com/JuliaRobotics/IncrementalInference.jl/issues/1007).

* Blank / white -- uninitialized or unprocessed,
* Orange -- recycled clique upsolve solution from previous tree passed into `solveTree!` -- TODO,
* Blue -- fully marginalized clique that will not be updated during upsolve (maybe downsolved),
* Light blue -- completed downsolve,
* Green -- trying to up initialize,
* Darkgreen -- `initUp` some could up init,
* Lightgreen -- `initUp` no aditional variables could up init,
* Olive -- trying to down initialize,
* Seagreen -- `initUp` some could down init,
* Khaki -- `initUp` no aditional variables could down init,
* Brown -- initialized but not solved yet (likely child cliques that depend on downward autoinit msgs),
* Light red -- completed upsolve,
* Tomato -- partial dimension upsolve but finished,
* Red -- CPU working on clique's Chapman-Kolmogorov inference (up),
* Maroon -- CPU working on clique's Chapman-Kolmogorov inference (down),
* Red -- If finished cliques in red are in `ERROR_STATUS`
