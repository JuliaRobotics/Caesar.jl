var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": "<p align=\"center\">\n<img src=\"assets/logo.png\" width=\"480\" border=\"0\" />\n</p>A modern robotic toolkit for localization and mapping – reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM).(Image: Caesar)Towards non-parametric / parametric state estimation and navigation solutions [1]. Implemented in Julia (and JuliaPro) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages like C/C++ or Python, as listed in features below. Multi-modal (quasi-multi-hypothesis) navigation and mapping solutions, using various sensor data, is a corner stone of this package. Multi-sensor fusion is made possible via vertically integrated Multi-modal iSAM.Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the FactorGraph through a graph database using CloudGraphs.jl, as discussed here[2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl][rome-url] and back-end solver [IncrementalInference.jl][iif-url].Comments, questions and issues welcome."
},

{
    "location": "index.html#Major-features-1",
    "page": "Home",
    "title": "Major features",
    "category": "section",
    "text": "Performing multi-core inference with Multi-modal iSAM over factor graphs, supporting Pose2, Pose3, Point2, Point3, Null hypothesis, Multi-modal, KDE density, partial constraints, and more.tree = wipeBuildBayesTree!(fg, drawpdf=true)\ninferOverTree!(fg, tree)Or directcly on a database, allowing for separation of concernsslamindb()Local copy of database held FactorGraphfg = Caesar.initfg(cloudGraph, session)\nfullLocalGraphCopy(fg)Saving and loading FactorGraph objects to filesavejld(fg, file=\"test.jld\", groundtruth=gt)\nloadjld(file=\"test.jld\")Visualization through MIT Director.visualizeallposes(fg) # from local dictionary\ndrawdbdirector()      # from database held factor graphFoveation queries to quickly organize, extract and work with big data blobs, for example looking at images from multiple sessions predicted to see the same point [-9.0,9.0] in the map:neoids, syms = foveateQueryToPoint(cloudGraph,[\"SESS21\";\"SESS38\";\"SESS45\"], point=[-9.0;9.0], fovrad=0.5 )\nfor neoid in neoids\n    cloudimshow(cloudGraph, neoid=neoid)\nendOperating on data from a thin client processes, such as a Python front-endexamples/database/python/neo_interact_example.jlA caesar-lcm server interface for C++ applications is available here.\nA multicore Bayes 2D feature tracking server over tcpjulia -p10 -e \"using Caesar; tcpStringBRTrackingServer()\"And many more, please see the examples folder."
},

{
    "location": "index.html#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": "Requires via sudo apt-get install, see DrakeVisualizer.jl for more details.libvtk5-qt4-dev python-vtkThen install required Julia packages  julia> Pkg.add(\"Caesar\")Note that Database related packages will not be automatically installed. Please see section below for details."
},

{
    "location": "index.html#Basic-usage-1",
    "page": "Home",
    "title": "Basic usage",
    "category": "section",
    "text": "Here is a basic example of using visualization and multi-core factor graph solving:addprocs(2)\nusing Caesar, RoME, TransformUtils, Distributions\n\n# load scene and ROV model (might experience UDP packet loss LCM buffer not set)\nvc = startdefaultvisualization()\nsc1 = loadmodel(:scene01); sc1(vc)\nrovt = loadmodel(:rov); rovt(vc)\n\n\ninitCov = 0.001*eye(6); [initCov[i,i] = 0.00001 for i in 4:6];\nodoCov = 0.0001*eye(6); [odoCov[i,i] = 0.00001 for i in 4:6];\nrangecov, bearingcov = 3e-4, 2e-3\n\n# start and add to a factor graph\nfg = identitypose6fg(initCov=initCov)\ntf = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )\naddOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf), odoCov) ) )\n\nvisualizeallposes!(vc, fg, drawlandms=false)\n\naddLinearArrayConstraint(fg, (4.0, 0.0), :x0, :l1, rangecov=rangecov,bearingcov=bearingcov)\nvisualizeDensityMesh!(vc, fg, :l1)\naddLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)\n\nsolveandvisualize(fg, vc, drawlandms=true, densitymeshes=[:l1;:x2])"
},

{
    "location": "index.html#Database-interaction-layer-1",
    "page": "Home",
    "title": "Database interaction layer",
    "category": "section",
    "text": "For using the solver on a Database layer, you simply need to switch the working API. This can be done by calling the database connection function, and following the prompt:using Caesar\nbackend_config, user_config = standardcloudgraphsetup()\nfg = Caesar.initfg(sessionname=user_config[\"session\"], cloudgraph=backend_config)\n# and then continue as normal with the fg object, to add variables and factors, draw etc.If you have access to Neo4j and Mongo services you should be able to run the four door test.Go to your browser at localhost:7474 and run one of the Cypher queries to either retrievematch (n) return nor delete everything:match (n) detach delete nYou can run the multi-modal iSAM solver against the DB using the example MM-iSAMCloudSolve.jl:$ julia -p20\njulia> using Caesar\njulia> slamindb() # iterations=-1Database driven Visualization can be done with either MIT's MIT Director (prefered), or Collections Render which additionally relies on Pybot. For visualization using Director/DrakeVisualizer.jl:$ julia -e \"using Caesar; drawdbdirector()\"And an example service script for CollectionsRender is also available."
},

{
    "location": "index.html#Contributors-1",
    "page": "Home",
    "title": "Contributors",
    "category": "section",
    "text": "D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. Leonard"
},

{
    "location": "index.html#Future-targets-1",
    "page": "Home",
    "title": "Future targets",
    "category": "section",
    "text": "This is a work in progress package. Please file issues here as needed to help resolve problems for everyone!Hybrid parametric and non-parametric optimization. Incrementalized update rules and properly marginalized 'forgetting' for sliding window type operation. We defined interprocess interface for multi-language front-end development."
},

{
    "location": "index.html#References-1",
    "page": "Home",
    "title": "References",
    "category": "section",
    "text": "[1]  Fourie, D.: \"Multi-modal and Inertial Sensor Solutions to Navigation-type Factor Graph\",\n     Ph.D. Thesis, Massachusetts Institute of Technology Electrical Engineering and Computer Science together with Woods Hole Oceanographic Institution Department for Applied Ocean Science and Engineering, September 2017.\n[2]  Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: \"SLAMinDB: Centralized graph\n     databases for mobile robotics\" IEEE International Conference on Robotics and Automation (ICRA),\n     Singapore, 2017."
},

{
    "location": "index.html#Manual-Outline-1",
    "page": "Home",
    "title": "Manual Outline",
    "category": "section",
    "text": "Pages = [\n    \"index.md\"\n    \"examples.md\"\n    \"func_ref.md\"\n]\nDepth = 3"
},

{
    "location": "examples.html#",
    "page": "Examples",
    "title": "Examples",
    "category": "page",
    "text": ""
},

{
    "location": "examples.html#Examples-1",
    "page": "Examples",
    "title": "Examples",
    "category": "section",
    "text": "Intersection of ambiguous elevation angle from planar SONAR sensor:   <a href=\"http://vimeo.com/198237738\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/dehann/Caesar.jl/master/docs/imgs/rovasfm02.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>Bi-modal belief   <a href=\"http://vimeo.com/198872855\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/dehann/Caesar.jl/master/docs/imgs/rovyaw90.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>Multi-session Turtlebot example of the second floor in the Stata Center:   <img src=\"https://raw.githubusercontent.com/dehann/Caesar.jl/master/docs/imgs/turtlemultisession.gif\" alt=\"Turtlebot Multi-session animation\" width=\"480\" border=\"0\" /></a>Multi-modal range only example:   <a href=\"http://vimeo.com/190052649\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/dehann/IncrementalInference.jl/master/doc/images/mmisamvid01.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>"
},

{
    "location": "tutorialcontinuousscalar.html#",
    "page": "Tutorial 1: ContinuousScalar",
    "title": "Tutorial 1: ContinuousScalar",
    "category": "page",
    "text": ""
},

{
    "location": "tutorialcontinuousscalar.html#Tutorials-1",
    "page": "Tutorial 1: ContinuousScalar",
    "title": "Tutorials",
    "category": "section",
    "text": ""
},

{
    "location": "tutorialcontinuousscalar.html#IncrementalInference.jl-ContinuousScalar-1",
    "page": "Tutorial 1: ContinuousScalar",
    "title": "IncrementalInference.jl ContinuousScalar",
    "category": "section",
    "text": "This tutorial illustrates how IncrementalInference enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information. This tutorial is rather abstract and the user is free to imagine any system of relationships, for example a robot driving in a one dimensional world, or a time traveler making uncertain jumps forwards and backwards in time. The tutorial implicitly shows a multi-modal uncertainty introduced and transmitted. The tutorial also illustrates consensus through an additional piece of information, which reduces all stochastic variable marginal beliefs to unimodal only beliefs. The example will also illustrate the use of non-Gaussian beliefs and global inference. The tutorial also shows how to create user defined functions. Lastly, the tutorial demonstrates how automatic initialization of variables works.This tutorial requires IncrementalInference v0.3.0+, RoME v0.1.0, RoMEPlotting packages be installed. In addition, the optional GraphViz package will allow easy visualization of the FactorGraph object structure.To start, the two major mathematical packages are brought into scope.using Distributions\nusing IncrementalInferenceThis tutorial calls for multiple variable nodes connected through algebraic functions stochastic uncertainty. User scope Prior, LinearOffset, and MultiModalOffset with arbitrary distributions are defined as:struct Prior{T} <: IncrementalInference.FunctorSingleton where T <: Distribution\n  z::T\nend\ngetSample(s::Prior, N::Int=1) = (reshape(rand(s.z,N),1,N), )\nstruct LinearOffset{T} <: IncrementalInference.FunctorPairwise where T <: Distribution\n  z::T\nend\ngetSample(s::LinearOffset, N::Int=1) = (rand(s.z,N), )\nfunction (s::LinearOffset)(res::Array{Float64},\n      idx::Int,\n      meas::Tuple{Array{Float64, 1}},\n      X1::Array{Float64,2},\n      X2::Array{Float64,2}  )\n  #\n  res[1] = meas[1][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nend\nstruct MultiModalOffset <: IncrementalInference.FunctorPairwise\n  z::Vector{Distribution}\n  c::Categorical\nend\ngetSample(s::MultiModalOffset, N::Int=1) = (rand.(s.z, N)..., rand(s.c, N))\nfunction (s::MultiModalOffset)(res::Array{Float64},\n      idx::Int,\n      meas::Tuple,\n      X1::Array{Float64,2},\n      X2::Array{Float64,2}  )\n  #\n  res[1] = meas[meas[3][idx]][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nendNotice the residual function relating to the two PairwiseFunctor derived definitions. The one dimensional residual functions, res[1] = measurement - prediction, are used during inference to approximate the convolution of conditional beliefs from the sample approximate marginal beliefs of the connected variables.Guidelines for developing your own functions are discussed here (TBD), and we note that mechanizations and manifolds required for robotic simultaneous localization and mapping (SLAM) has been tightly integrated with the expansion package RoME.jl.The next step is to describe the inference problem with a graphical model of type IncrementalInference.FactorGraph. The first step is to create an empty factor graph object and start populating it with variable nodes. The variable nodes are identified by Symbols, namely :x0, :x1, :x2, :x3.# Start with an empty factor graph\nfg = emptyFactorGraph()\n\n# add the first node\naddNode!(fg, :x0, ContinuousScalar)\n\n# this is unary (prior) factor and does not immediately trigger autoinit of :x0.\naddFactor!(fg, [:x0], Prior(Normal(0,1)))Factor graphs are bipartite graphs with factors that act as mathematical structure between interacting variables. After adding node :x0, a singleton factor of type Prior (which was defined by the user earlier) is 'connected to' variable node :x0. This unary factor is taken as a Distributions.Normal distribution with zero mean and a standard devitation of 1. GraphViz.jl can be used to visualize the factor graph structure, although the package is not installed by default. Furthermore, the writeGraphPdf member definition is given at the end of this tutorial, which allows the user to store the graph image in graphviz supported image types.Graphs.plot(fg.g)\n# writeGraphPdf(fg, file=\"fgx01.pdf\") # file=\"fgx01.png\"The two node factor graph is shown in the image below.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0.png\" width=\"120\" border=\"0\" />\n</p>Automatic initialization of variables depend on how the factor graph model is constructed. This tutorial demonstrates this behavior by first showing that :x0 is not initialized:@show isInitialized(fg, :x0)Why is :x0 not initialized? Since no other variable nodes have been 'connected to' (or depend) on :x0 and future intentions of the user are unknown, the initialization of :x0 is deferred until the latest possible moment. IncrementalInference.jl assumes that the user will generally populate new variable nodes with most of the associated factors before moving to the next variable. By delaying initialization of a new variable (say :x0) until a second newer uninitialized variable (say :x1) depends on :x0, the IncrementalInference algorithms hope to then initialize :x0 with the more information from previous and surrounding variables and factors. Also note that initialization of variables is a local operation based only on the neighboring nodes – global inference will over the entire graph is shows later in this tutorial.By adding :x1 and connecting it through the LinearOffset and Normal distributed factor, the automatic initialization of :x0 is triggered.addNode!(fg, :x1, ContinuousScalar)\n# P(Z | :x1 - :x0 ) where Z ~ Normal(10,1)\naddFactor!(fg, [:x0, :x1], LinearOffset(Normal(10.0,1)))\n@show isInitialized(fg, :x0) # trueNote that the automatic initialization of :x0 is aware that :x1 is not initialized and therefore only used the Prior(Normal(0,1)) unary factor to initialize the marginal belief estimate for :x0. The structure of the graph has now been updated to two variable nodes and two factors.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx01.png\" width=\"240\" border=\"0\" />\n</p>Global inference requires that the entire factor graph be initialized before the numerical belief computation algorithms can be performed. Notice how the new :x1 variable is not yet initialized:@show isInitialized(fg, :x1) # falseThe RoMEPlotting.jl package allows visualization (plotting) of the belief state over any of the variable nodes. Remember the first time executions are slow given required code compilation, and that future versions of these package will use more precompilation to reduce first execution running cost.using RoMEPlotting\n\nplotKDE(fg, :x0)<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0.png\" width=\"360\" border=\"0\" />\n</p>By forcing the initialization of :x1 and plotting its belief estimate,ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1])the predicted influence of the P(Z| X1 - X0) = LinearOffset(Normal(10, 1)) is shown by the red trace.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx01.png\" width=\"360\" border=\"0\" />\n</p>The red trace (predicted belief of :x1) is noting more than the approximated convolution of the current marginal belief of :x0 with the conditional belief described by P(Z | X1 - X0).Another ContinuousScalar variable :x2 is 'connected' to :x1 through a more complicated MultiModalOffset likelihood function.addNode!(fg, :x2, ContinuousScalar)\nmmo = MultiModalOffset([Rayleigh(3); Uniform(30,55)], Categorical([0.4; 0.6]))\naddFactor!(fg, [:x1, :x2], mmo)<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx012.png\" width=\"360\" border=\"0\" />\n</p>The mmo variable illustrates how a near arbitrary mixture probability distribution can be used as a conditional relationship between variable nodes in the factor graph. In this case, a 40%/60% balance of a Rayleigh and truncated Uniform distribution which acts as a multi-modal conditional belief. Interpret carefully what a conditional belief of this nature actually means.Following the tutorial's practical example frameworks (robot navigation or time travel), this multi-modal belief implies that moving from one of the probable locations in :x1 to a location in :x2 by some processes defined by mmo=P(Z | X2, X1) is uncertain to the same 40%/60% ratio. In practical terms, collapsing (through observation of an event) the probabilistic likelihoods of the transition from :x1 to :x2 may result in the :x2 location being at either 15-20, or 40-65-ish units. The predicted belief over :x2 is illustrated by plotting the predicted belief (green trace), after forcing initialization.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2])<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx012.png\" width=\"360\" border=\"0\" />\n</p>Adding one more variable :x3 through another LinearOffset(Normal(-50,1))addNode!(fg, :x3, ContinuousScalar)\naddFactor!(fg, [:x2, :x3], LinearOffset(Normal(-50, 1)))expands the factor graph to to four variables and four factors.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0123.png\" width=\"480\" border=\"0\" />\n</p>This part of the tutorial shows how a unimodal likelihood (conditional belief) can transmit the bimodal belief currently contained in :x2.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2, :x3])Notice the blue trace (:x3) is a shifted and slightly spread out version of the initialized belief on :x2, through the convolution with the conditional belief P(Z | X2, X3).<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0123.png\" width=\"480\" border=\"0\" />\n</p>Global inference over the entire factor graph has still not occurred, and will at this stage produce roughly similar results to the predicted beliefs shown above. Only by introducing more information into the factor graph can inference extract more precise marginal belief estimates for each of the variables. A final piece of information added to this graph is a factor directly relating :x3 with :x0.addFactor!(fg, [:x3, :x0], LinearOffset(Normal(40, 1)))Pay close attention to what this last factor means in terms of the probability density traces shown in the previous figure. The blue trace for :x3 has two major modes, one that overlaps with :x0, :x1 near 0 and a second mode further to the left at -40. The last factor introduces a shift LinearOffset(Normal(40,1)) which essentially aligns the left most mode of :x3 back onto :x0. This last factor forces a mode selection through consensus. By doing global inference, the new information obtained in :x3 will be equally propagated to :x2 where only one of the two modes will remain.Global inference is achieved with local computation using two function calls, as follows.tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# and visualization\nplotKDE(fg, [:x0, :x1, :x2, :x3])The resulting posterior marginal beliefs over all the system variables are:<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0123infr.png\" width=\"480\" border=\"0\" />\n</p>It is import to note that although this tutorial ends with all marginal beliefs having near Gaussian shape and are unimodal, that the package supports multi-modal belief estimates during both the prediction and global inference processes. In fact, many of the same underlying inference functions are involved with the automatic initialization process and the global multi-modal iSAM inference procedure. This concludes the ContinuousScalar tutorial particular to the IncrementalInference package."
},

{
    "location": "func_ref.html#",
    "page": "Functions",
    "title": "Functions",
    "category": "page",
    "text": ""
},

{
    "location": "func_ref.html#Function-Reference-1",
    "page": "Functions",
    "title": "Function Reference",
    "category": "section",
    "text": "Pages = [\n    \"func_ref.md\"\n]\nDepth = 3"
},

{
    "location": "func_ref.html#Caesar.loadmodel",
    "page": "Functions",
    "title": "Caesar.loadmodel",
    "category": "Function",
    "text": "mdl = loadmodel(model, [color=, offset=])\n\nBrings model data into context, but generating or loading meshes, Geometries, etc as required by requested model, and must still be drawn with the Visualizer object using mdl(vis).\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getPoseExVertexNeoIDs",
    "page": "Functions",
    "title": "Caesar.getPoseExVertexNeoIDs",
    "category": "Function",
    "text": "getPoseExVertexNeoIDs(neo4j.connection)\n\nReturn array of tuples with ExVertex IDs and Neo4j IDs for all poses.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.askneo4jcredentials!",
    "page": "Functions",
    "title": "Caesar.askneo4jcredentials!",
    "category": "Function",
    "text": "askneo4jcredentials!(;addrdict::Dict{AbstractString, AbstractString})\n\nObtain Neo4j global database address and login credientials from STDIN, then insert and return in the addrdict colletion.\n\n\n\n"
},

{
    "location": "func_ref.html#RoME.getRangeKDEMax2D",
    "page": "Functions",
    "title": "RoME.getRangeKDEMax2D",
    "category": "Function",
    "text": "getRangeKDEMax2D(cgl::CloudGraph, session::AbstractString, vsym1::Symbol, vsym2::Symbol)\n\nCalculate the cartesian distange between two vertices in the graph, by session and symbol names, and by maximum belief point.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getLandmOtherSessNeoIDs",
    "page": "Functions",
    "title": "Caesar.getLandmOtherSessNeoIDs",
    "category": "Function",
    "text": "getLandmOtherSessNeoIDs{T <: AbstractString}(::CloudGraph, session::T=\"\", multisessions=Vector{T}())\n\nReturn dict of dict of Neo4j vertex IDs by session and landmark symbols.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.db2jld",
    "page": "Functions",
    "title": "Caesar.db2jld",
    "category": "Function",
    "text": "db2jld(cgl::CloudGraph, session::AbstractString, filename::AbstractString)\n\nFetch and save a FactorGraph session to a jld, using CloudGraph object and session definition.\n\n\n\ndb2jld(filename::AbstractString; addrdict::VoidUnion{Dict{AbstractString, AbstractString}}=nothing )\n\nFetch and save a FactorGraph session to a jld, using or asking STDIN for credentials in the addrdict field.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.identitypose6fg",
    "page": "Functions",
    "title": "Caesar.identitypose6fg",
    "category": "Function",
    "text": "identitypose6fg()\n\nInitialize a and exiting or new factor graph with a first pose and prior as specified by default keywords or user.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.fetchrobotdatafirstpose",
    "page": "Functions",
    "title": "Caesar.fetchrobotdatafirstpose",
    "category": "Function",
    "text": "fetchrobotdatafirstpose(cg::CloudGraph, session::AbstractString)\n\nReturn dict of JSON parsed \"robot_description\" field as was inserted by counterpart insertrobotdatafirstpose! function. Used for storing general robot specific data in easily accessible manner.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.executeQuery",
    "page": "Functions",
    "title": "Caesar.executeQuery",
    "category": "Function",
    "text": "executeQuery(cg::CloudGraph, query::AbstractString)\n\nRun Neo4j Cypher queries on the cloudGraph database, andreturn Tuple with the unparsed (results, loadresponse).\n\n\n\n"
},

{
    "location": "func_ref.html#IncrementalInference.ls",
    "page": "Functions",
    "title": "IncrementalInference.ls",
    "category": "Function",
    "text": "ls(cgl::CloudGraph, session::AbstractString; sym::Symbol=, neoid::Int=,exvid::Int=)\n\nList neighbors to node in cgl::CloudGraph by returning Dict{Sym}=(exvid, neoid, Symbol[labels]), and can take any of the three as input node identifier. Not specifying an identifier will result in all Variable nodes being returned.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.updateparallelposes!",
    "page": "Functions",
    "title": "Caesar.updateparallelposes!",
    "category": "Function",
    "text": "updateparallelposes!(vis, poseswithdepth, wTb=::CoordinateTransformations.AbstractAffineMap)\n\nUpdate all triads listed in poseswithdepth[Symbol(vert.label)] with wTb. Prevents cycles in remote tree viewer of DrakeVisualizer.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.removeReinsertMultisessionPrior!",
    "page": "Functions",
    "title": "Caesar.removeReinsertMultisessionPrior!",
    "category": "Function",
    "text": "removeReinsertMultisessionPrior!{T <: FunctorSingleton}(fgl::FactorGraph, exims::Dict{Symbol, Int}, prp2::T, sym::Symbol, uid::Int)\n\nReturn new multisession factor, symbol sym, inserted using prp2::PriorPoint2DensityNH using user ExVertex id (uid), after checking existing multisession dict (exims::(exvsym=>neoid)) if any nodes should be removed before inserting new ones on that graph node. Assuming just one multisession prior per node.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.appendvertbigdata!",
    "page": "Functions",
    "title": "Caesar.appendvertbigdata!",
    "category": "Function",
    "text": "appendvertbigdata!(cloudGraph, cloudvert, descr, data)\n\nAppend big data element into current blob store and update associated global vertex information.\n\n\n\nappendvertbigdata!(fg, vert, descr, data)\n\nAppend big data element into current blob store and update associated global vertex information.\n\n\n\nappendvertbigdata!(fg, sym, descr, data)\n\nAppend big data element into current blob store using parent appendvertbigdata!, but here specified by symbol of variable node in the FactorGraph. Note the default data layer api definition. User must define dlapi to refetching the  vertex from the data layer. localapi avoids repeated network database fetches.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.parseMergeVertAttr!",
    "page": "Functions",
    "title": "Caesar.parseMergeVertAttr!",
    "category": "Function",
    "text": "parseMergeVertAttr(v, elem)\n\nParse elem dictionary according to thin frtend interface from other languages, and merge contents into attributes of v::Graphs.ExVertex.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.updatenewverts!",
    "page": "Functions",
    "title": "Caesar.updatenewverts!",
    "category": "Function",
    "text": "updatenewverts!(fgl::FactorGraph; N::Int)\n\nConvert vertices of session in Neo4j DB with Caesar.jl's required data elements in preparation for MM-iSAMCloudSolve process.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.mergeCloudVertex!",
    "page": "Functions",
    "title": "Caesar.mergeCloudVertex!",
    "category": "Function",
    "text": "mergeCloudVertex!(...)\n\nHodgepodge function to merge data in CloudVertex\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getVertNeoIDs!",
    "page": "Functions",
    "title": "Caesar.getVertNeoIDs!",
    "category": "Function",
    "text": "getVertNeoIDs!(::CloudGraph, res::Dict{Symbol, Int}; session::AbstractString=\"NA\")\n\nInsert into and return dict res with Neo4j IDs of ExVertex labels as stored per session in Neo4j database.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.whosNear3D",
    "page": "Functions",
    "title": "Caesar.whosNear3D",
    "category": "Function",
    "text": "whosNear3D(cg::CloudGraph, session::AbstractString; x,y,z,roll, pitch,yaw,dist,angle )\n\nFind vertices near the point specified and return dictionary of symbol to Neo4j ID pairs.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getprpt2kde",
    "page": "Functions",
    "title": "Caesar.getprpt2kde",
    "category": "Function",
    "text": "getprp2kde(::CloudGraph, neoids::Vector{Int}; N::Int=100)\n\nReturn PriorPoint2DensityNH with N points based on beliefs of neoids, and equal share null hypothesis between length(neoids)+1 beliefs.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.whosNear2D",
    "page": "Functions",
    "title": "Caesar.whosNear2D",
    "category": "Function",
    "text": "whosNear2D(cg::CloudGraph, session::AbstractString; x,y,yaw,dist,angle )\n\nFind vertices near the point specified and return dictionary of symbol to Neo4j ID pairs.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.hasBigDataElement",
    "page": "Functions",
    "title": "Caesar.hasBigDataElement",
    "category": "Function",
    "text": "hasBigDataElement(vertex, description)\n\nReturn true if vertex has bigDataElements with matching description.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getAllLandmarkNeoIDs",
    "page": "Functions",
    "title": "Caesar.getAllLandmarkNeoIDs",
    "category": "Function",
    "text": "getAllLandmarkNeoIDs(::Dict{Symbol, Dict{Symbol, Int}}, ::Symbol)\n\nReturn Vector{Int} of Neo4j vertex IDs relating to symbol, as listed in lm2others.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.consoleaskuserfordb",
    "page": "Functions",
    "title": "Caesar.consoleaskuserfordb",
    "category": "Function",
    "text": "consoleaskuserfordb(;nparticles=false, drawdepth=false, clearslamindb=false)\n\nObtain database addresses and login credientials from STDIN, as well as a few case dependent options.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.drawAllOdometryEdges!",
    "page": "Functions",
    "title": "Caesar.drawAllOdometryEdges!",
    "category": "Function",
    "text": "drawAllOdometryEdges!(fgl::FactorGraph, fr::Symbol, to::Symbol; scale, color, api  )\n\nAssume odometry chain and draw edges between subsequent poses. Use keyword arguments to change colors, etc.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.standardcloudgraphsetup",
    "page": "Functions",
    "title": "Caesar.standardcloudgraphsetup",
    "category": "Function",
    "text": "standardcloudgraphsetup(;addrdict=nothing, nparticles=false, drawdepth=false, clearslamindb=false)\n\nConnect to databases via network according to addrdict, or ask user for credentials and return active cloudGraph object, as well as addrdict.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.resetentireremotesession",
    "page": "Functions",
    "title": "Caesar.resetentireremotesession",
    "category": "Function",
    "text": "resetentireremotesession(conn, session)\n\nmatch (n:session) remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width set n :NEWDATA return n\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getfirstpose",
    "page": "Functions",
    "title": "Caesar.getfirstpose",
    "category": "Function",
    "text": "getfirstpose(cg::CloudGraph, session::AbstractString)\n\nReturn Tuple{Symbol, Int} of first pose symbol and Neo4j node ID.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getExVertexNeoIDs",
    "page": "Functions",
    "title": "Caesar.getExVertexNeoIDs",
    "category": "Function",
    "text": "getExVertexNeoIDs(neo4j.connection, label=\"\", session=\"\")\n\nReturn array of tuples with ExVertex IDs and Neo4j IDs for vertices with label in session.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.findExistingMSConstraints",
    "page": "Functions",
    "title": "Caesar.findExistingMSConstraints",
    "category": "Function",
    "text": "findExistingMSConstraints(fgl::FactorGraph)\n\nReturn Dict{Symbol, Int} of vertex symbol to Neo4j node ID of MULTISESSION constraints in this fgl.sessionname.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.removeNeo4jID",
    "page": "Functions",
    "title": "Caesar.removeNeo4jID",
    "category": "Function",
    "text": "removeNeo4jID(cg::CloudGraph, neoid=-1)\n\nRemove node from Neo4j according to Neo4j Node ID. Big data elements that may be associated with this node are not removed.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.drawLineBetween!",
    "page": "Functions",
    "title": "Caesar.drawLineBetween!",
    "category": "Function",
    "text": "drawLineBetweenPose3(fr::Graphs.ExVertex, to::Graphs.ExVertex; scale=, color=  )\n\nDraw a line segment between to vertices.\n\n\n\ndrawLineBetween(fgl::FactorGraph, fr::Symbol, to::Symbol; scale, color, api  )\n\nDraw a line segment between to nodes in the factor graph.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getBigDataElement",
    "page": "Functions",
    "title": "Caesar.getBigDataElement",
    "category": "Function",
    "text": "getBigDataElement(vertex::CloudVertex, description)\n\nWalk through vertex bigDataElements and return the last matching description.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.askmongocredentials!",
    "page": "Functions",
    "title": "Caesar.askmongocredentials!",
    "category": "Function",
    "text": "askmongocredentials!(addrdict=Dict{AbstractString, AbstractString})\n\nObtain Mongo database address and login credientials from STDIN, then insert and return in the addrdict colletion.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.rmInstMultisessionPriors!",
    "page": "Functions",
    "title": "Caesar.rmInstMultisessionPriors!",
    "category": "Function",
    "text": "rmInstMultisessionPriors!{T <: AbstractString}(::CloudGraph; session::T=, multisessions:Vector{T}= )\n\n\n\n"
},

{
    "location": "func_ref.html#RoME.getLastPose",
    "page": "Functions",
    "title": "RoME.getLastPose",
    "category": "Function",
    "text": "getLastPose(cg::CloudGraph, session::AbstractString)\n\nReturn Tuple{Symbol, Int} of first pose symbol and Neo4j node ID.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.insertrobotdatafirstpose!",
    "page": "Functions",
    "title": "Caesar.insertrobotdatafirstpose!",
    "category": "Function",
    "text": "insertrobotdatafirstpose!(cg::CloudGraph, session::AbstractString, robotdict::Dict)\n\nSaves robotdict via JSON to first pose in a SESSION in the database. Used for storing general robot specific data in easily accessible manner. Can fetch later retrieve same dict with counterpart fetchrobotdatafirstpose function.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.fetchsubgraph!",
    "page": "Functions",
    "title": "Caesar.fetchsubgraph!",
    "category": "Function",
    "text": "fetchsubgraph!(::FactorGraph, ::Vector{CloudVertex}, numneighbors::Int=0)\n\nFetch and insert list of CloudVertices into FactorGraph object, up to neighbor depth.\n\n\n\nfetchsubgraph!(::FactorGraph, ::Vector{Int}, numneighbors::Int=0)\n\nFetch and insert list of Neo4j IDs into FactorGraph object, up to neighbor depth.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getLocalSubGraphMultisession",
    "page": "Functions",
    "title": "Caesar.getLocalSubGraphMultisession",
    "category": "Function",
    "text": "getLocalSubGraphMultisession{T <: AbstractString}(cg::CloudGraph, lm2others; session::T=\"\", numneighbors::Int=0)\n\nReturn subgraph copy of type FactorGraph contaning values from session in lm2others, and Vector{Symbol} of primary key symbols used for graph exstraction.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar.getnewvertdict",
    "page": "Functions",
    "title": "Caesar.getnewvertdict",
    "category": "Function",
    "text": "getnewvertdict(conn, session)\n\nReturn a dictionary with frtend and mongo_keys json string information for :NEWDATA elements in Neo4j database.\n\n\n\n"
},

{
    "location": "func_ref.html#Caesar-1",
    "page": "Functions",
    "title": "Caesar",
    "category": "section",
    "text": "    Caesar.loadmodel\n    Caesar.getPoseExVertexNeoIDs\n    Caesar.askneo4jcredentials!\n    RoME.getRangeKDEMax2D\n    Caesar.getLandmOtherSessNeoIDs\n    Caesar.db2jld\n    Caesar.identitypose6fg\n    Caesar.fetchrobotdatafirstpose\n    Caesar.executeQuery\n    IncrementalInference.ls\n    Caesar.updateparallelposes!\n    Caesar.removeReinsertMultisessionPrior!\n    Caesar.appendvertbigdata!\n    Caesar.parseMergeVertAttr!\n    Caesar.updatenewverts!\n    Caesar.mergeCloudVertex!\n    Caesar.getVertNeoIDs!\n    Caesar.whosNear3D\n    Caesar.getprpt2kde\n    Caesar.whosNear2D\n    Caesar.hasBigDataElement\n    Caesar.getAllLandmarkNeoIDs\n    Caesar.consoleaskuserfordb\n    Caesar.drawAllOdometryEdges!\n    Caesar.standardcloudgraphsetup\n    Caesar.resetentireremotesession\n    Caesar.getfirstpose\n    Caesar.getExVertexNeoIDs\n    Caesar.findExistingMSConstraints\n    Caesar.removeNeo4jID\n    Caesar.drawLineBetween!\n    Caesar.getBigDataElement\n    Caesar.askmongocredentials!\n    Caesar.rmInstMultisessionPriors!\n    RoME.getLastPose\n    Caesar.insertrobotdatafirstpose!\n    Caesar.fetchsubgraph!\n    Caesar.getLocalSubGraphMultisession\n    Caesar.getnewvertdict"
},

{
    "location": "func_ref.html#Index-1",
    "page": "Functions",
    "title": "Index",
    "category": "section",
    "text": ""
},

]}
