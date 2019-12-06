
# Advanced Topics on Bayes Tree

## Definitions

Squashing or collapsing the Bayes tree back into a 'flat' Bayes net, by chain rule: 

```math
p(x,y) = p(x|y)p(y) = p(y|x)p(x) \\
p(x,y,z) = p(x|y,z)p(y,z) = p(x,y|z)p(z) = p(x|y,z)p(y|z)p(z) \\
p(x,y,z) = p(x|y,z)p(y)p(z) \, \text{iff y is independent of z,} \, also p(y|z)=p(y)
```

## Are cliques in the Bayes (Junction) tree densly connected?

Yes and no. From the chordal Bayes net's perspective (obtained through the elimination game in order to build the clique tree), the nodes of the Bayes tree are indeed fully connected subgraphs (they are called cliques after all!). From the perspective of the subgraph of the original factor graph induced by the clique's variables, cliques need not be fully connected, since we are assuming the factor graph as sparse, and that no new information can be created out of nothing---hence each clique must be sparse.  That said, the potential exists for the inference within a clique to become densly connected (experience full "fill-in").  See the paper on square-root-SAM, where the connection between dense covariance matrix of a Kalman filter (EKF-SLAM) is actually related to the inverse square root (rectangular) matrix which structure equivalent to the clique subgraph adjacency matrix.  

Also remember that the intermediate Bayes net (which has densly connected cliques) hides the underlying tree structure -- think of the Bayes net as looking at the tree from on top or below, thereby encoding the dense connectivity in the structure of the tree itself.  All information below any clique of the tree is encoded in the upward marginal belief messages at that point (i.e. the densly connected aspects pertained lower down in the tree).


## LU/QR vs. Belief Propagation

LU/QR is a special case (Parametric/Linear) of more general belief propagation.  The story though is more intricate, where QR/LU assume that product-factors can be formed through the chain rule -- using congruency -- it is not that straight forward with general beliefs.  In the general case we are almost forced to use belief propagation, which in turn implies special care is needed to describe the relationship between sparse factor graph fragments in cliques on the tree, and the more densely connected structure of the Bayes Net.

## Bayes Tree vs Bayes Net

The Bayes tree is a purely symbolic structure -- i.e. special grouping of factors that all come from the factor graph joint product (product of independently sampled likelihood/conditional models):

```math
[\Theta | Z] \propto \prod_i \, [ Z_i=z_i | \Theta_i ]
```

A sparse factor graph problem can be squashed into smaller dense problem of product-factor conditionals (from variable elimination).  Therefore each product-factor (aka "smart factor" in other uses of the language) represent both the factors as well as the sequencing of cliques in that branch.  This process repeats recursively from the root down to the leaves.  The leaves of the tree have no further reduced product factors condensing child cliques below, and therefore sparse factor fragments can be computed to start the upward belief propagation process.  More importantly, as belief propagation progresses up the tree, upward belief messages (on clique separators) capture the same structure as the densely connected Bayes net but each clique in the Bayes tree still only contains sparse fragments from the original factor graph.  The structure of the tree (combined parent-child relationships) encodes the same information as the product-factor conditionals!

## Initialization on the Tree

It more challenging but possible to initialize all variables in a factor graph through belief propagation on the Bayes tree.

As a thought experiment: Wouldn't it be awesome if we could compile the upsolve as a symbolic process only, and only assign numerical values once during a single downsolve procedure.  The origin of this idea comes from the realization that a complete upsolve on the Bayes (Junction) tree is very nearly the same thing finding good numerical initialization values for the factor graph.  If the up-init-solve can be performed as a purely symbolic process, it would greatly simplify numerical computations by deferring them to the down solve alone.

Trying to do initialization for real, we might want to replace up-init-symbolic operations with numerical equivalents.  Either way, it would be worth knowing what the equivalent numerical operations of a full up-init-solve of an uninitialized factor graph would look like.

In general, if a clique can not be initialized based on information from lower down in that branch of the tree; more information is need from the parent.  In the Gaussian (more accurately the congruent factor) case, all information lower down in the branch---i.e. the relationships between variables in parent---can be summarized by a new conditional product-factor that is computed with the probabilistic chain rule.  To restate, the process of squashing the Bayes tree branch back down into a Bayes net, is effectively the the chain rule process used in variable elimination.

!!! note

    Cascading up and down solves are required if you do not use eliminated factor conditionals in parent cliques.


### Gaussian-only special case

Elimination of variables and factors using chain rule reduction is a special case of belief propagation, and thus far only the reduction of congruent beliefs (such as Gaussian) is known.

These computations can be parallelized depending on the conditional independence structure of the Bayes tree -- separate branches are effectively separate chain rule instances.  This is precisely the same process exploited by multi-frontal QR matrix factorization.

On the down solve the conditionals---from eliminated chains of previously eliminated variables and factors---can be used for inference directly in the parent.  

See node x1 to x3 in IncrementalInference issue 464. It does not branch or provide additional prior information. so it is collapsed into one factor between x1 and x3, solved in the root and the individual variable can be solved by inference.


!!! note

    What does the Jacobian in Gaussian only case mean with regard to a symbolic upsolve?
