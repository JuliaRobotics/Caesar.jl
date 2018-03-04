var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": "<p align=\"center\">\n<img src=\"assets/logo.png\" width=\"480\" border=\"0\" />\n</p>A modern robotic toolkit for localization and mapping – reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM).(Image: Caesar)Towards non-parametric / parametric state estimation and navigation solutions [1]. Implemented in Julia (and JuliaPro) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages like C/C++ or Python, as listed in features below. Multi-modal (quasi-multi-hypothesis) navigation and mapping solutions, using various sensor data, is a corner stone of this package. Multi-sensor fusion is made possible via vertically integrated Multi-modal iSAM.Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the FactorGraph through a graph database using CloudGraphs.jl, as discussed here[2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl][rome-url] and back-end solver [IncrementalInference.jl][iif-url].Comments, questions and issues welcome."
},

{
    "location": "index.html#Contributors-1",
    "page": "Home",
    "title": "Contributors",
    "category": "section",
    "text": "Authors directly involved with this package are:D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. LeonardWe are grateful for many, many contributions within the Julia package ecosystem – see the REQUIRE files of Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs and others for a far reaching list of contributions."
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
    "text": "Caesar.jl is registered with the regular Julia METADATA and can be installed as follows:julia> Pkg.add(\"Caesar\")Please note that visualizations have been moved to the Arena.jl package and documentation can be found on the visualization page of this documentation."
},

{
    "location": "index.html#Basic-usage-1",
    "page": "Home",
    "title": "Basic usage",
    "category": "section",
    "text": "The basic example has been moved to the visualization page."
},

{
    "location": "index.html#Future-targets-1",
    "page": "Home",
    "title": "Future targets",
    "category": "section",
    "text": "This is a work in progress package. Please file issues here as needed to help resolve problems for everyone!Hybrid parametric and non-parametric optimization. Incrementalized update rules and properly marginalized \'forgetting\' for sliding window type operation. We defined interprocess interface for multi-language front-end development."
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
    "text": "This tutorial illustrates how IncrementalInference enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information. This tutorial is rather abstract and the user is free to imagine any system of relationships, for example a robot driving in a one dimensional world, or a time traveler making uncertain jumps forwards and backwards in time. The tutorial implicitly shows a multi-modal uncertainty introduced and transmitted. The tutorial also illustrates consensus through an additional piece of information, which reduces all stochastic variable marginal beliefs to unimodal only beliefs. The example will also illustrate the use of non-Gaussian beliefs and global inference. The tutorial also shows how to create user defined functions. Lastly, the tutorial demonstrates how automatic initialization of variables works.This tutorial requires IncrementalInference v0.3.0+, RoME v0.1.0, RoMEPlotting packages be installed. In addition, the optional GraphViz package will allow easy visualization of the FactorGraph object structure.To start, the two major mathematical packages are brought into scope.using Distributions\nusing IncrementalInferenceThis tutorial calls for multiple variable nodes connected through algebraic functions stochastic uncertainty. User scope Prior, LinearOffset, and MultiModalOffset with arbitrary distributions are defined as:import IncrementalInference: getSample\n\nstruct Prior{T} <: IncrementalInference.FunctorSingleton where T <: Distribution\n  z::T\nend\ngetSample(s::Prior, N::Int=1) = (rand(s.z,N), )\nstruct LinearOffset{T} <: IncrementalInference.FunctorPairwise where T <: Distribution\n  z::T\nend\ngetSample(s::LinearOffset, N::Int=1) = (rand(s.z,N), )\nfunction (s::LinearOffset)(res::Array{Float64},\n      idx::Int,\n      meas::Tuple{Array{Float64, 1}},\n      X1::Array{Float64,2},\n      X2::Array{Float64,2}  )\n  #\n  res[1] = meas[1][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nend\nstruct MultiModalOffset <: IncrementalInference.FunctorPairwise\n  z::Vector{Distribution}\n  c::Categorical\nend\ngetSample(s::MultiModalOffset, N::Int=1) = (rand.(s.z, N)..., rand(s.c, N))\nfunction (s::MultiModalOffset)(res::Array{Float64},\n      idx::Int,\n      meas::Tuple,\n      X1::Array{Float64,2},\n      X2::Array{Float64,2}  )\n  #\n  res[1] = meas[meas[end][idx]][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nendNotice the residual function relating to the two PairwiseFunctor derived definitions. The one dimensional residual functions, res[1] = measurement - prediction, are used during inference to approximate the convolution of conditional beliefs from the sample approximate marginal beliefs of the connected variables.Guidelines for developing your own functions are discussed here (TBD), and we note that mechanizations and manifolds required for robotic simultaneous localization and mapping (SLAM) has been tightly integrated with the expansion package RoME.jl.The next step is to describe the inference problem with a graphical model of type IncrementalInference.FactorGraph. The first step is to create an empty factor graph object and start populating it with variable nodes. The variable nodes are identified by Symbols, namely :x0, :x1, :x2, :x3.# Start with an empty factor graph\nfg = emptyFactorGraph()\n\n# add the first node\naddNode!(fg, :x0, ContinuousScalar)\n\n# this is unary (prior) factor and does not immediately trigger autoinit of :x0.\naddFactor!(fg, [:x0], Prior(Normal(0,1)))Factor graphs are bipartite graphs with factors that act as mathematical structure between interacting variables. After adding node :x0, a singleton factor of type Prior (which was defined by the user earlier) is \'connected to\' variable node :x0. This unary factor is taken as a Distributions.Normal distribution with zero mean and a standard devitation of 1. GraphViz.jl can be used to visualize the factor graph structure, although the package is not installed by default. Furthermore, the writeGraphPdf member definition is given at the end of this tutorial, which allows the user to store the graph image in graphviz supported image types.Graphs.plot(fg.g)\n# writeGraphPdf(fg, file=\"fgx01.pdf\") # file=\"fgx01.png\"The two node factor graph is shown in the image below.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0.png\" width=\"120\" border=\"0\" />\n</p>Automatic initialization of variables depend on how the factor graph model is constructed. This tutorial demonstrates this behavior by first showing that :x0 is not initialized:@show isInitialized(fg, :x0) # falseWhy is :x0 not initialized? Since no other variable nodes have been \'connected to\' (or depend) on :x0 and future intentions of the user are unknown, the initialization of :x0 is deferred until the latest possible moment. IncrementalInference.jl assumes that the user will generally populate new variable nodes with most of the associated factors before moving to the next variable. By delaying initialization of a new variable (say :x0) until a second newer uninitialized variable (say :x1) depends on :x0, the IncrementalInference algorithms hope to then initialize :x0 with the more information from previous and surrounding variables and factors. Also note that initialization of variables is a local operation based only on the neighboring nodes – global inference will over the entire graph is shows later in this tutorial.By adding :x1 and connecting it through the LinearOffset and Normal distributed factor, the automatic initialization of :x0 is triggered.addNode!(fg, :x1, ContinuousScalar)\n# P(Z | :x1 - :x0 ) where Z ~ Normal(10,1)\naddFactor!(fg, [:x0, :x1], LinearOffset(Normal(10.0,1)))\n@show isInitialized(fg, :x0) # trueNote that the automatic initialization of :x0 is aware that :x1 is not initialized and therefore only used the Prior(Normal(0,1)) unary factor to initialize the marginal belief estimate for :x0. The structure of the graph has now been updated to two variable nodes and two factors.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx01.png\" width=\"240\" border=\"0\" />\n</p>Global inference requires that the entire factor graph be initialized before the numerical belief computation algorithms can be performed. Notice how the new :x1 variable is not yet initialized:@show isInitialized(fg, :x1) # falseThe RoMEPlotting.jl package allows visualization (plotting) of the belief state over any of the variable nodes. Remember the first time executions are slow given required code compilation, and that future versions of these package will use more precompilation to reduce first execution running cost.using RoMEPlotting\n\nplotKDE(fg, :x0)<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0.png\" width=\"360\" border=\"0\" />\n</p>By forcing the initialization of :x1 and plotting its belief estimate,ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1])the predicted influence of the P(Z| X1 - X0) = LinearOffset(Normal(10, 1)) is shown by the red trace.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx01.png\" width=\"360\" border=\"0\" />\n</p>The red trace (predicted belief of :x1) is noting more than the approximated convolution of the current marginal belief of :x0 with the conditional belief described by P(Z | X1 - X0).Another ContinuousScalar variable :x2 is \'connected\' to :x1 through a more complicated MultiModalOffset likelihood function.addNode!(fg, :x2, ContinuousScalar)\nmmo = MultiModalOffset([Rayleigh(3); Uniform(30,55)], Categorical([0.4; 0.6]))\naddFactor!(fg, [:x1, :x2], mmo)<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx012.png\" width=\"360\" border=\"0\" />\n</p>The mmo variable illustrates how a near arbitrary mixture probability distribution can be used as a conditional relationship between variable nodes in the factor graph. In this case, a 40%/60% balance of a Rayleigh and truncated Uniform distribution which acts as a multi-modal conditional belief. Interpret carefully what a conditional belief of this nature actually means.Following the tutorial\'s practical example frameworks (robot navigation or time travel), this multi-modal belief implies that moving from one of the probable locations in :x1 to a location in :x2 by some processes defined by mmo=P(Z | X2, X1) is uncertain to the same 40%/60% ratio. In practical terms, collapsing (through observation of an event) the probabilistic likelihoods of the transition from :x1 to :x2 may result in the :x2 location being at either 15-20, or 40-65-ish units. The predicted belief over :x2 is illustrated by plotting the predicted belief (green trace), after forcing initialization.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2])<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx012.png\" width=\"360\" border=\"0\" />\n</p>Adding one more variable :x3 through another LinearOffset(Normal(-50,1))addNode!(fg, :x3, ContinuousScalar)\naddFactor!(fg, [:x2, :x3], LinearOffset(Normal(-50, 1)))expands the factor graph to to four variables and four factors.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0123.png\" width=\"480\" border=\"0\" />\n</p>This part of the tutorial shows how a unimodal likelihood (conditional belief) can transmit the bimodal belief currently contained in :x2.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2, :x3])Notice the blue trace (:x3) is a shifted and slightly spread out version of the initialized belief on :x2, through the convolution with the conditional belief P(Z | X2, X3).<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0123.png\" width=\"480\" border=\"0\" />\n</p>Global inference over the entire factor graph has still not occurred, and will at this stage produce roughly similar results to the predicted beliefs shown above. Only by introducing more information into the factor graph can inference extract more precise marginal belief estimates for each of the variables. A final piece of information added to this graph is a factor directly relating :x3 with :x0.addFactor!(fg, [:x3, :x0], LinearOffset(Normal(40, 1)))Pay close attention to what this last factor means in terms of the probability density traces shown in the previous figure. The blue trace for :x3 has two major modes, one that overlaps with :x0, :x1 near 0 and a second mode further to the left at -40. The last factor introduces a shift LinearOffset(Normal(40,1)) which essentially aligns the left most mode of :x3 back onto :x0.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0123c.png\" width=\"480\" border=\"0\" />\n</p>This last factor forces a mode selection through consensus. By doing global inference, the new information obtained in :x3 will be equally propagated to :x2 where only one of the two modes will remain.Global inference is achieved with local computation using two function calls, as follows.tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# and visualization\nplotKDE(fg, [:x0, :x1, :x2, :x3])The resulting posterior marginal beliefs over all the system variables are:<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0123infr.png\" width=\"480\" border=\"0\" />\n</p>It is import to note that although this tutorial ends with all marginal beliefs having near Gaussian shape and are unimodal, that the package supports multi-modal belief estimates during both the prediction and global inference processes. In fact, many of the same underlying inference functions are involved with the automatic initialization process and the global multi-modal iSAM inference procedure. This concludes the ContinuousScalar tutorial particular to the IncrementalInference package."
},

{
    "location": "arena_visualizations.html#",
    "page": "Arena Visualization",
    "title": "Arena Visualization",
    "category": "page",
    "text": ""
},

{
    "location": "arena_visualizations.html#Visualization-with-Arena.jl-1",
    "page": "Arena Visualization",
    "title": "Visualization with Arena.jl",
    "category": "section",
    "text": "Caesar.jl uses the Arena.jl package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl. Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.Over time, Caesar.jl has used a least three different 3D visualization technologies, with the most recent based on WebGL and three.js by means of the MeshCat.jl package. The previous incarnation used a client side installation of VTK  by means of the DrakeVisualizer.jl and Director libraries. Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base. Each epoch has aimed at reducing dependencies and increasing multi-platform support.The sections below discuss 2D and 3D visualization techniques available to the Caesar.jl robot navigation system. Visualization examples will be seen throughout the Caesar.jl package documentation.Note that all visualizations used to be part of the Caesar.jl package itself, but was separated out to Arena.jl in early. There may be some package documentation glitches where the using Arena dependency has not been added – please file issues or suggest changes accordingly."
},

{
    "location": "arena_visualizations.html#Installation-1",
    "page": "Arena Visualization",
    "title": "Installation",
    "category": "section",
    "text": "This package will soon be registed with Julia METADATA which will make it available with the standard package management tools.  Within Julia or (JuliaPro) type:julia> Pkg.add(\"Arena\")Depending on the system, the following sudo apt-get install packages may be required, see DrakeVisualizer.jl for more details:libvtk5-qt4-dev python-vtk"
},

{
    "location": "arena_visualizations.html#D-Visualization-1",
    "page": "Arena Visualization",
    "title": "2D Visualization",
    "category": "section",
    "text": "2D plot visualizations are generally useful for repeated analysis of a algorithm or data set being studied. These visualizations are often manipulated to emphasize particular aspects of mobile platform navigation. Arena.jl is intended to simplify the process 2D plotting for robot trajectories in two or three dimensions. The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets."
},

{
    "location": "arena_visualizations.html#Hexagonal-2D-SLAM-example-visualization-1",
    "page": "Arena Visualization",
    "title": "Hexagonal 2D SLAM example visualization",
    "category": "section",
    "text": "This simplest example for visualizing a 2D robot trajectory is:using RoME, Arena  \n\nfg = initfg()\n\n# also add a PriorPose2 to pin the first pose at a fixed location\naddNode!(fg, :x0, Pose2, labels=[\"VARIABLE\";\"POSE\"])\naddFactor!(fg, [:x0], PriorPose2(zeros(3,1), 0.01*eye(3), [1.0]))\n\n# Drive around in a hexagon\nfor i in 0:5\n  psym = Symbol(\"x$i\")\n  nsym = Symbol(\"x$(i+1)\")\n  addNode!(fg, nsym, Pose2, labels=[\"VARIABLE\";\"POSE\"])\n  addFactor!(fg, [psym;nsym], Pose2Pose2(reshape([10.0;0;pi/3],3,1), 0.01*eye(3), [1.0]), autoinit=true )\n  # Pose2Pose2_NEW(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))\nend\n\nensureAllInitialized!(fg)\nsolveBatch!(fg)\n\n# The RoME and IncrementalInference\n\n# RoMEPlotting, KernelDensityEstimatePlotting and Gadfly packages provide the 2D visualization\ndrawPoses(fg)"
},

{
    "location": "arena_visualizations.html#D-Visualization-2",
    "page": "Arena Visualization",
    "title": "3D Visualization",
    "category": "section",
    "text": "Factor graphs of two or three dimensions can be visualized with the 3D visualizations provided by Arena.jl and it\'s dependencies. The 2D example above and also be visualized in a 3D space with the commands:vc = startdefaultvisualization() # to load a DrakeVisualizer/Director process instance\nvisualize(fg, vc, drawlandms=false)\n# visualizeallposes!(vc, fg, drawlandms=false)Here is a basic example of using visualization and multi-core factor graph solving:addprocs(2)\nusing Caesar, RoME, TransformUtils, Distributions\n\n# load scene and ROV model (might experience UDP packet loss LCM buffer not set)\nsc1 = loadmodel(:scene01); sc1(vc)\nrovt = loadmodel(:rov); rovt(vc)\n\ninitCov = 0.001*eye(6); [initCov[i,i] = 0.00001 for i in 4:6];\nodoCov = 0.0001*eye(6); [odoCov[i,i] = 0.00001 for i in 4:6];\nrangecov, bearingcov = 3e-4, 2e-3\n\n# start and add to a factor graph\nfg = identitypose6fg(initCov=initCov)\ntf = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )\naddOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf), odoCov) ) )\n\naddLinearArrayConstraint(fg, (4.0, 0.0), :x0, :l1, rangecov=rangecov,bearingcov=bearingcov)\naddLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)\n\nsolveBatch!(fg)\n\nusing Arena\n\nvc = startdefaultvisualization()\nvisualize(fg, vc, drawlandms=true, densitymeshes=[:l1;:x2])\nvisualizeDensityMesh!(vc, fg, :l1)\n# visualizeallposes!(vc, fg, drawlandms=false)"
},

{
    "location": "database_interactions.html#",
    "page": "Database Layer",
    "title": "Database Layer",
    "category": "page",
    "text": ""
},

{
    "location": "database_interactions.html#Database-interaction-layer-1",
    "page": "Database Layer",
    "title": "Database interaction layer",
    "category": "section",
    "text": "For using the solver on a Database layer, you simply need to switch the working API. This can be done by calling the database connection function, and following the prompt:using Caesar\nbackend_config, user_config = standardcloudgraphsetup()\nfg = Caesar.initfg(sessionname=user_config[\"session\"], cloudgraph=backend_config)\n# and then continue as normal with the fg object, to add variables and factors, draw etc.If you have access to Neo4j and Mongo services you should be able to run the four door test.Go to your browser at localhost:7474 and run one of the Cypher queries to either retrievematch (n) return nor delete everything:match (n) detach delete nYou can run the multi-modal iSAM solver against the DB using the example MM-iSAMCloudSolve.jl:$ julia -p20\njulia> using Caesar\njulia> slamindb() # iterations=-1Database driven Visualization can be done with either MIT\'s MIT Director (prefered), or Collections Render which additionally relies on Pybot. For visualization using Director/DrakeVisualizer.jl:$ julia -e \"using Caesar; drawdbdirector()\"And an example service script for CollectionsRender is also available."
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
