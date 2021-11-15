## Relative Factors (Legacy)
### One Dimension Roots Example

Previously we looked at adding a prior.  This section demonstrates the first of two `<:AbstractRelative` factor types.  These are factors that introduce only relative information between variables in the factor graph.

This example is on `<:IIF.AbstractRelativeRoots`.  First, lets create the factor as before 
```julia
struct MyFactor{T <: SamplableBelief} <: IIF.AbstractRelativeRoots
  Z::T
end
getSample(cfo::CalcFactor{<:MyFactor}, N::Int=1) = (reshape(rand(cfo.factor.Z,N) ,1,N), )

function (cfo::CalcFactor{<:MyFactor})( measurement_z,
                                        x1,
                                        x2  )
  #
  res = measurement_z - (x2[1] - x1[1])
  return res
end
```


The selection of `<:IIF.AbstractRelativeRoots`, akin to earlier `<:AbstractPrior`, instructs IIF to find the roots of the provided residual function.  That is the one dimensional residual function, `res[1] = measurement - prediction`, is used during inference to approximate the convolution of conditional beliefs from the approximate beliefs of the connected variables in the factor graph.

Important aspects to note, `<:IIF.AbstractRelativeRoots` requires all elements `length(res)` (the factor measurement dimension) to have a feasible zero crossing solution.  A two dimensional system will solve for variables where both `res[1]==0` and `res[2]==0`.

!!! note
    As of IncrementalInference v0.21, CalcResidual no longer takes a residual as input parameter and should return residual, see IIF#467.

!!! note
    Measurements and variables passed in to the factor residual function do not have the same type as when constructing the factor graph.  It is recommended to leave these incoming types unrestricted.  If you must define the types, these either are (or will be) of element type relating to the manifold on which the measurement or variable beliefs reside.  Probably a vector or manifolds type.  Usage can be very case specific, and hence better to let Julia type-inference automation do the hard work for you. The 

### Two Dimension Minimize Example

The second type is `<:IIF.AbstractRelativeMinimize` which simply minimizes the residual vector of the user factor. This type is useful for partial constraint situations where the residual function is not gauranteed to have zero crossings in all dimensions and the problem is converted into a minimization problem instead:
```julia
struct OtherFactor{T <: SamplableBelief} <: IIF.AbstractRelativeMinimize
  Z::T             # assuming something 2 dimensional
  userdata::String # or whatever is necessary
end

# just illustrating some arbitraty second value in tuple of different size
getSample(cfo::CalcFactor{<:OtherFactor}, N::Int=1) = (rand(cfo.factor.z,N), rand())

function (cfo::CalcFactor{<:OtherFactor})(res::AbstractVector{<:Real},
                                          z,
                                          second_val,
                                          x1,
                                          x2 )
  #
  # @assert length(z) == 2
  # not doing anything with `second_val` but illustrating
  # not doing anything with `cfo.factor.userdata` either
  
  # the broadcast operators with automatically vectorize
  res = z .- (x1[1:2] .- x1[1:2])
  return res
end
```
