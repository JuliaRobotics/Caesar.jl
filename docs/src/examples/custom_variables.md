
## Creating New Variables

A handy macro can help define new variables:
```@docs
@defVariable
```

All variables have to derive from `IncrementalInference.InferenceVariable`.

What you need to build in the variable:
* `dims` - This is used during computation and defines the degrees of freedom (dimensions) for variable
* `labels` - This a required field, although it does not need to be populated. It consists of unique, indexable string identifiers, such as 'POSE', 'LANDMARK'. It assists with querying the data efficiently in large systems when using the database layer.  

You can then also add any additional fields that you would like to use for saving state information in variable. Note that these fields must be serializable as both JSON and Protobufs. Although you don't need to validate this, please keep the fields fairly simple and avoid complex structures with optional fields. TBD - provide a compatibility check for serialization and a docpage on it.

In a trivial example of Pose2:
* Our dimensions would then be 3: X, Y, theta
* The labels for Pose2 could be "POSE"

!!! note
    See [RoME.jl#244](http://www.github.com/JuliaRobotics/RoME.jl/issues/244) regarding plans to fundamentally integrate with [Manifolds.jl](http://www.github.com/JuliaManifolds/Manifolds.jl)

