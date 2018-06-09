## IncrementalInference.jl Defining Factors (2017 API)

This tutorial describes how a new factor can be developed, beyond the pre-existing implementation in [RoME.jl](http://www.github.com/JuliaRobotics/RoME.jl).  Factors can accept any number of variable dependencies and allow for a wide class of allowable function calls can be used.  Our intention is to make it as easy as possible for users to create their own factor types.

A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models -- for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion [here (author disclosure)](https://darchive.mblwhoilibrary.org/bitstream/handle/1912/9305/Fourie_thesis.pdf?sequence=1):
![factorgraphexample](https://user-images.githubusercontent.com/6412556/41196136-e5b05f98-6c07-11e8-8f26-7318e5085cc0.png)


### Example: Adding Velocity to Point2D

A smaller example in two dimensions where we wish to estimate the velocity of some target:  Consider two variables `:x0` with a prior as well as a conditional---likelihood for short---to variable `:x1`.  Priors are in the "global" reference frame (how ever you choose to define it), while likelihoods are in the "local" / "relative" frame that only exist between variables.

![dynpoint2fg](https://user-images.githubusercontent.com/6412556/40951628-caf1d332-6845-11e8-9710-9f6fcd92a8ca.png)


## IncrementalInference.jl Defining Factors (Future API)

We would like to remove the `idx` indexing from the residual function calls, since that is an unnecessary burden on the user.  Instead, the package will use `views` and `SubArray` types to simplify the interface.  Please contact author for more details (8 June 2018).
