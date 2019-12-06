
# Advanced Topics on Bayes Tree

## Definitions

- Squashing or collapsing the Bayes tree back into a 'flat' Bayes net,
- Chain rule: p(x,y) = p(x|y)p(y) = p(y|x)p(x)
  - p(x,y,z) = p(x|y,z)p(y,z) = p(x,y|z)p(z) = p(x|y,z)p(y|z)p(z)
  - p(x,y,z) = p(x|y,z)p(y)p(z) iff y is independent of z === p(y|z)=p(y)

## LU/QR vs. Belief Propagation

LU/QR is a special case (Parametric/Linear) of more general belief propagation.  The story though is more intricate, where QR/LU assume that product-factors can be formed through the chain rule -- using congruency -- it is not that straight forward with general beliefs.  In the general case we are almost forced to use belief propagation, which in turn implies special care is needed to describe the relationship between sparse factor graph fragments in cliques on the tree, and the more densely connected structure of the Bayes Net.

## Bayes Tree vs Bayes Net

The Bayes tree is a purely symbolic structure -- i.e. special grouping of factors that all come from the factor graph joint product.

FULL JOINT PRODUCT HERE

A sparse factor graph problem can be squashed into smaller dense problem of product-factor conditionals (from variable elimination).  Therefore each product-factor (aka "smart factor" in other uses of the language) represent both the factors as well as the sequencing of cliques in that branch.  This process repeats recursively from the root down to the leaves.  The leaves of the tree have no further reduced product factors condensing child cliques below, and therefore sparse factor fragments can be computed to start the upward belief propagation process.  More importantly, as belief propagation progresses up the tree, upward belief messages (on clique separators) capture the same structure as the densely connected Bayes net but each clique in the Bayes tree still only contains sparse fragments from the original factor graph.

## Initialization on the Tree

It more challenging but possible to initialize all variables in a factor graph through belief propagation on the Bayes tree.

As a thought experiment: Wouldn't it be awesome if we could compile the upsolve as a symbolic process only, and only assign numerical values once during a single downsolve procedure.  The origin of this idea comes from the realization that a complete upsolve on the Bayes (Junction) tree is very nearly the same thing finding good numerical initialization values for the factor graph.  If the up-init-solve can be performed as a purely symbolic process, it would greatly simplify numerical computations by deferring them to the down solve alone.

Trying to do initialization for real, we might want to replace up-init-symbolic operations with numerical equivalents.  Either way, it would be worth knowing what the equivalent numerical operations of a full up-init-solve of an uninitialized factor graph would look like.

Cascading up and down solves are required if you do not use eliminated factor conditionals in parent cliques.



## Gaussian-only special case

Elimination of variables and factors using chain rule reduction is a special case of belief propagation, and thus far only the reduction of congruent beliefs (such as Gaussian) is known.

Just like the non-Gaussian case, if a clique can not be initialized based on information from lower down in that branch of the tree; more information is need from the parent.  In the Gaussian (more accurately the congruent factor) case, all information lower down in the branch---i.e. the relationships between variables in parent---can be summarized by a new conditional factor that is computed with the probabilistic chain rule.  To restate, the process of squashing the Bayes tree branch back down into a Bayes net, is effectively the the chain rule process used in variable elimination.

These computations can be parallelized depending on the conditional independence structure of the Bayes tree -- separate branches are effectively separate chain rule instances.  This is precisely the same process exploited by multi-frontal QR matrix factorization.

On the down solve the conditionals---from eliminated chains of previously eliminated variables and factors---can be used for inference directly in the parent.  

See node x1 to x3 in IncrementalInference issue 464. It does not branch or provide additional prior information. so it is collapsed into one factor between x1 and x3, solved in the root and the individual variable can be solved by inference.




note:

  What does the Jacobian in Gaussian only case mean with regard to a symbolic upsolve?
