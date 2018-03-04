## Database interaction layer
---

For using the solver on a Database layer, you simply need to switch the working API. This can be done by calling the database connection function, and following the prompt:

```julia
using Caesar
backend_config, user_config = standardcloudgraphsetup()
fg = Caesar.initfg(sessionname=user_config["session"], cloudgraph=backend_config)
# and then continue as normal with the fg object, to add variables and factors, draw etc.
```

If you have access to Neo4j and Mongo services you should be able to run the [four door test](https://github.com/dehann/Caesar.jl/blob/master/test/fourdoortestcloudgraph.jl).

Go to your browser at localhost:7474 and run one of the Cypher queries to either retrieve

    match (n) return n

or delete everything:

    match (n) detach delete n

You can run the multi-modal iSAM solver against the DB using the example [MM-iSAMCloudSolve.jl](https://github.com/dehann/Caesar.jl/blob/master/examples/database/MM-iSAMCloudSolve.jl):
```
$ julia -p20
julia> using Caesar
julia> slamindb() # iterations=-1
```

Database driven Visualization can be done with either MIT's [MIT Director](https://github.com/rdeits/DrakeVisualizer.jl) (prefered), or Collections Render which additionally relies on [Pybot](http://www.github.com/spillai/pybot). For visualization using Director/DrakeVisualizer.jl:
```
$ julia -e "using Caesar; drawdbdirector()"
```

And an [example service script for CollectionsRender](https://github.com/dehann/Caesar.jl/blob/master/examples/database/DBCollectionsViewerService.jl) is also available.
