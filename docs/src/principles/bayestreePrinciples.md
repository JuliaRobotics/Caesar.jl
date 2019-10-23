# Principle: Bayes tree prototyping

This page describes how to visualize, study, test, and compare Bayes (Junction) tree concepts with special regard for variable ordering.

**Under construction**

## What is a Bayes (Junction) tree

REQUEST Tonio

## Constructing a Tree

Trees and factor graphs are separated in the implementation, allowing the user to construct multiple different trees from one factor graph except for a few temporary values in the factor graph.

```julia
using IncrementalInference # RoME or Caesar will work too

## construct a distributed factor graph object
fg = initfg()
# add variables and factors
# ...

## build the tree
tree = wipeBuildNewTree!(fg)
```

The temporary values are `wiped` from the distributed factor graph object `fg<:AbstractDFG` and a new tree is constructed.  This `wipeBuildNewTree!` call can be repeated as many times the user desires and results should be consistent for the same factor graph structure (regardless of numerical values contained within).

## Visualizing

IncrementalInference.jl includes functions for visualizing the Bayes tree, and uses outside packages such as GraphViz (standard) and Latex tools (experimental, optional) to do so.  

### GraphViz

```julia
drawTree(tree, show=true) # , filepath="/tmp/caesar/mytree.pdf"
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

### Visualizing Clique Adjacency Matrix

It is also possible to see the upward message passing variable/factor association matrix for each clique, requiring the Gadfly.jl package:
```julia
using Gadfly

spyCliqMat(tree, :x1) # provided by IncrementalInference

#or embedded in graphviz
drawTree(tree, imgs=true, show=true
```

## Variable Ordering

### Getting the AMD Variable Ordering

The variable ordering is described as a `::Vector{Symbol}`.
```julia
vo = getEliminationOrder(fg)
tree = buildTreeFromOrdering!(fg, vo)
```
The temporary elimination values in `fg` can be reset with (currently rather aggressive):
```julia
resetFactorGraphNewTree!(fg)
```

These steps are combined in a wrapper function:
```julia
resetBuildTreeFromOrder!(fg, vo)
```

### Manipulating the Variable Ordering

```julia
vo = [:x1; :l3; :x2; ...]
```

> **Note** that a list of variables or factors can be obtained through the `ls` and related functions:

Variables:
```julia
unsorted = ls(fg)
unsorted = ls(fg, Pose2) # by variable type
unsorted = ls(fg, r"x")  # by regex
unsorted = intersect(ls(fg, r"x"), ls(fg, Pose2))  # by regex

# sorting
sorted = sortDFG(unsorted)  # deprecated name sortVarNested(unsorted)
```

Factors:
```julia
unsorted = lsf(fg)
unsorted = ls(fg, Pose2Point2BearingRange)
```

## Interfacing with 'mmisam' Solver

The regular solver used in IIF is:
```julia
tree, smt, hist = solveTree!(fg)
```
where a new tree is constructed internally.  In order to recycle computations from a previous tree, the following interface can be used:
```julia
tree, smt, hist = solveTree!(fg, tree)
```
which will replace the `tree` object pointer to the new tree object after solution.
