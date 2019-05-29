## Frequently Asked Questions

### Why Julia
The [JuliaLang](https://julialang.org/) and ([JuliaPro](https://juliacomputing.com/)) is an open-source Just-In-Time (JIT) & optionally precompiled, strongly-typed, and high-performance programming language.
The algorithmic code is implemented in Julia for many reasons, such as agile development, high level syntax, performance, type safety, parallel computing, dynamic development, cross compilable (with gcc and clang) and foundational cross-platform ([LLVM](http:///www.llvm.org)) technologies.  
See [JuliaCon2018 highlights video](https://www.youtube.com/watch?v=baR02tlea5Y).  Julia can be thought of as either {C+, Mex (done right), or as a modern Fortran replacement}.  

### Is Caesar.jl limited to Julia? No.
The Caesar.jl project is expressly focused on making this algorithmic code available to [C/Fortran](https://docs.julialang.org/en/v1/manual/calling-c-and-fortran-code/)/[C++](https://juliacomputing.com/blog/2017/12/01/cxx-and-cxxwrap-intro.html)/C#/[Python](https://github.com/JuliaPy/PyCall.jl)/[Java](https://github.com/JuliaInterop/JavaCall.jl)/JS.  Julia itself offers [many additional interops](https://github.com/JuliaInterop).  ZMQ and HTTP/WebSockets are the standardized interfaces of choice, please see [details at the multi-language section](../concepts/multilang/)).  Consider opening issues or getting in touch for more information.

### Can Julia be Embedded into C/C++
Yes, see [the Julia embedding documentation page](https://docs.julialang.org/en/v1/manual/embedding/index.html).

### Current Julia version, v1.0.x
Caesar.jl and packages are currently [targeting Julia v1.0.x](https://julialang.org/downloads/) (2019Q1).  See [progress for Julia v1.1.x here](https://github.com/JuliaRobotics/Caesar.jl/issues/299).

### Just-In-Time Compiling (i.e. why are first runs slow?)
Julia uses just-in-time compilation ([unless already pre-compiled](https://stackoverflow.com/questions/40116045/why-is-julia-taking-a-long-time-on-the-first-call-into-my-module))
 which takes additional time the first time a new function is called. Additional calls to a function is fast from the second call onwards since the static function is now cached and ready for use.

### Static, Shared Object `.so` Compilation

Packages are already compiled to static objects (`.ji` files), but can also be compiled to more common `.so` files.  See [this AOT vs JIT compiling blog post](https://juliacomputing.com/blog/2016/02/09/static-julia.html) for a deeper discussion.  Also see [this Julia Binaries Blog](https://medium.com/@sdanisch/compiling-julia-binaries-ddd6d4e0caf4).

> **Note** [recent developments announced on discourse.](https://discourse.julialang.org/t/ann-packagecompiler-with-incremental-system-images/20489).  Also see new brute force sysimg work at [Fezzik.jl](https://github.com/TsurHerman/Fezzik).


### ROS Integration

ROS integration is a priority for this project and will accompany the so-called ['prime time'](https://github.com/JuliaRobotics/RoME.jl/issues/147) release of the code.  ROS and ZMQ interfaces are closely related.

> **Note** the present focus (2018Q3-2019Q2) is to stabilize the [ZMQ interface](../concepts/multilang).

> **Voice** Please add your voice of support or suggestions on [ROS integration here](https://github.com/JuliaRobotics/Caesar.jl/issues/227).

### How does JSON-Schema work?

Caesar.jl intends to follow [json-schema.org](http://www.json-schema.org), see [step-by-step guide here](https://json-schema.org/learn/getting-started-step-by-step.html).


## Solver FAQ (mm-iSAM)

### Why Bayes (Juntion) tree?

We want to perform inference on acyclic graphs, as well as exploit the benefits of knowing the full conditional independence structure of the graph---trees represent the "complete form" when marginalizing each variable one at a time (also known as elimination game, marginalization, or smart factors).  In loose terms---and assume if the system were to be linearlized parametric---think of the Bayes (Junction) tree as having implicit access to all Schur complements of each variable to all others.

### Are cliques in the Bayes (Junction) tree densly connected?

Yes and no. From the chordal Bayes net's perspective (obtained throught the elimination game in order to build the clique tree), the nodes of the Bayes tree are indeed fully connected subgraphs (they are called cliques after all!). From the perspective of the subgraph of the original factor graph induced by the clique's variables, cliques need not be fully connected, since we are assuming the factor graph as sparse, and that no new information can be created out of nothing---hence each clique must be sparse.  That said, the potential exists for the inference within a clique to become densly connected (experience full "fill-in").  See the paper on square-root-SAM, where the connection between dense covariance matrix of a Kalman filter (EKF-SLAM) is actually related to the inverse square root (rectangular) matrix which structure equivalent to the clique subgraph adjacency matrix.

### Why/Where does non-Gaussian data come from?

Gaussian error models in measurement or data cues will only be Gaussian (normally distributed) if all physics/decisions/systematic-errors/calibration/etc. has a correct algebraic model in every single circumstance.  Caesar.jl and mm-iSAM is heavily focussed on state-estimation from a plethora of heterogenous data.  Four major categories of non-Gaussian errors have thus far been considered:
- Uncertain decisions (a.k.a. data association), such as a robot trying to decide if a navigation loop-closure can be deduced from a repeat observation of a similar object or measurement from current and past data.  These issues are commonly also referred to as multi-hypothesis.
- Under-determined or under-defined systems where there are more variables than constraining measurements to fully define the system as a single mode---a.k.a solution ambiguity.  For example, in 2D consider two range measurements resulting in two possible locations through trilateration.
- Physics of the measurement process.  Many, if not all measurement processes exhibit non-Gaussian behaviour.  For example, acoustic/radio time-of-flight measurements, using either pulse-train or matched filtering, result in an "energy intensity" over time/distance of what the range to a scattering-target/source might be--i.e. highly non-Gaussian.
- Nonlinearity.  For example in 2D, consider a Pose2 odometry where the orientation is uncertain:  The resulting belief of where a next pose might be (convolution with odometry factor) results in a banana shape curve, even though the entire process is driven by assumed Gaussian belief.
