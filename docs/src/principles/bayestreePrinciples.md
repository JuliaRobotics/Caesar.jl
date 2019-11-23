# Principle: Bayes tree prototyping

This page describes how to visualize, study, test, and compare Bayes (Junction) tree concepts with special regard for variable ordering.

## Why a Bayes (Juntion) tree

The tree is algebraicly equivalent---but acyclic---structure to the factor graph.  Inference is easier on on acyclic graphs.  
We can also exploit Smart Message Passing benefits (known from the full full conditional independence structure encoded in the tree), since the tree represents the "complete form" when marginalizing each variable one at a time (also known as elimination game, marginalization, also related to smart factors).  In loose terms, the Bayes (Junction) tree has implicit access to all Schur complements (if it parametric and linearized) of each variable to all others.

## What is a Bayes (Junction) tree

The Bayes tree data structure is a rooted and directed Junction tree (maximal elimination clique tree). It allows for exact inference to be carried out by leveraging and exposing the variables' conditional independence and, very interestingly, can be directly associated with the sparsity pattern exhibited by a system's factorized upper triangular square root information matrix (see picture below).

![graph and matrix analagos](https://user-images.githubusercontent.com/27132241/69210533-f55da400-0b52-11ea-89dd-f18b7fa983b8.png)

Following this matrix-graph parallel, the picture also shows what the associated matrix interpretation is for a factor graph (~first order expansion in the form of a measurement Jacobian) and its corresponding Markov random field (sparsity pattern corresponding to the information matrix).

The procedure for obtaining the Bayes (Junction) tree is outlined in the figure shown below (factor graph to chrodal Bayes net via bipartite elimination game, and chordal Bayes net to Bayes tree via maximum cardinality search algorithm).

![add the fg2net2tree outline](https://user-images.githubusercontent.com/27132241/69210647-5eddb280-0b53-11ea-82ab-dc5ff89c4a43.png)

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
An example Bayes (Junction) tree representation obtained through `generateTexTree(tree)` for the sample factor graph shown above can be seen in the following image.

![add the fg2net2tree outline](https://user-images.githubusercontent.com/27132241/69210722-9e0c0380-0b53-11ea-9462-7964844b89b1.png)

### Visualizing Clique Adjacency Matrix

It is also possible to see the upward message passing variable/factor association matrix for each clique, requiring the Gadfly.jl package:
```julia
using Gadfly

spyCliqMat(tree, :x1) # provided by IncrementalInference

#or embedded in graphviz
drawTree(tree, imgs=true, show=true)
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
which will replace the `tree` object pointer to the new tree object after solution.  The following parmaters (set before calling `solveTree!`) will show the solution progress on the tree visualization:
```julia
getSolverParams(fg).drawtree = true
getSolverParams(fg).showtree = true
```
The color legend is currently recorded in an [issue thread here](https://github.com/JuliaRobotics/IncrementalInference.jl/issues/349).


### Are cliques in the Bayes (Junction) tree densly connected?

Yes and no. From the chordal Bayes net's perspective (obtained through the elimination game in order to build the clique tree), the nodes of the Bayes tree are indeed fully connected subgraphs (they are called cliques after all!). From the perspective of the subgraph of the original factor graph induced by the clique's variables, cliques need not be fully connected, since we are assuming the factor graph as sparse, and that no new information can be created out of nothing---hence each clique must be sparse.  That said, the potential exists for the inference within a clique to become densly connected (experience full "fill-in").  See the paper on square-root-SAM, where the connection between dense covariance matrix of a Kalman filter (EKF-SLAM) is actually related to the inverse square root (rectangular) matrix which structure equivalent to the clique subgraph adjacency matrix.  

Also remember that the intermediate Bayes net (which has densly connected cliques) hides the underlying tree structure -- think of the Bayes net as looking at the tree from on top or below, thereby encoding the dense connectivity in the structure of the tree itself.  All information below any clique of the tree is encoded in the upward marginal belief messages at that point (i.e. the densly connected aspects pertained lower down in the tree).

### Clique State Machine

The mmisam solver is based on a state machine design to handle the inter and intra clique operations during a variety of situations.  Use of the clique state machine (CSM) makes debugging, development, verification, and modification of the algorithm real easy.  Contact us for any support regarding modifications to the default algorithm.  For pre-docs on working with CSM, please see [IIF #443](https://github.com/JuliaRobotics/IncrementalInference.jl/issues/443).
