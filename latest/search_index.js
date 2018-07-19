var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": "<p align=\"center\">\n<img src=\"assets/logo.png\" width=\"480\" border=\"0\" />\n</p>A modern robotic toolkit for localization and mapping – reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM).(Image: Caesar)Towards non-parametric / parametric state estimation and navigation solutions [1]. Implemented in Julia (and JuliaPro) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages like C/C++ or Python, as listed in features below. Multi-modal (quasi-multi-hypothesis) navigation and mapping solutions, using various sensor data, is a corner stone of this package. Multi-sensor fusion is made possible via vertically integrated Multi-modal iSAM.Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the FactorGraph through a graph database using CloudGraphs.jl, as discussed here[2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate [RoME.jl][rome-url] and back-end solver [IncrementalInference.jl][iif-url].Comments, questions and issues welcome."
},

{
    "location": "index.html#Dependency-Structure-1",
    "page": "Home",
    "title": "Dependency Structure",
    "category": "section",
    "text": "IncrementalInference.jl supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself. RoME.jl introduces nodes and factors that are useful to robotic navigation. RoMEPlotting.jl are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D.Caesar.jl is the umbrella repo that depends on RoME.jl and others to support that \'passes through\' the same functionality while introducing more. For example, interaction with database server systems, LCMCore.jl, (future ROS support), and more.Arena.jl is a collection of 3D visualization tools and also depends on RoMEPlotting.jl for 2D visualizations.In the future, Caesar.jl would likely interact more closely with repo\'s such as SensorFeatureTracking.jl, AprilTags.jl, and RecursiveFiltering.jl"
},

{
    "location": "index.html#Major-features-1",
    "page": "Home",
    "title": "Major features",
    "category": "section",
    "text": "Performing multi-core inference with Multi-modal iSAM over factor graphs, supporting Pose2, Pose3, Point2, Point3, Null hypothesis, Multi-modal, KDE density, partial constraints, and more.tree = wipeBuildBayesTree!(fg, drawpdf=true)\ninferOverTree!(fg, tree)Or directcly on a database, allowing for separation of concernsslamindb()Local copy of database held FactorGraphfg = Caesar.initfg(cloudGraph, session)\nfullLocalGraphCopy(fg)Saving and loading FactorGraph objects to filesavejld(fg, file=\"test.jld\", groundtruth=gt)\nloadjld(file=\"test.jld\")Visualization through Arena.jl.visualizeallposes(fg) # from local dictionary\ndrawdbdirector()      # from database held factor graphFoveation queries to quickly organize, extract and work with big data blobs, for example looking at images from multiple sessions predicted to see the same point [-9.0,9.0] in the map:neoids, syms = foveateQueryToPoint(cloudGraph,[\"SESS21\";\"SESS38\";\"SESS45\"], point=[-9.0;9.0], fovrad=0.5 )\nfor neoid in neoids\n    cloudimshow(cloudGraph, neoid=neoid)\nendOperating on data from a thin client processes, such as a Python front-endexamples/database/python/neo_interact_example.jlA caesar-lcm server interface for C++ applications is available here.\nA multicore Bayes 2D feature tracking server over tcpjulia -p10 -e \"using Caesar; tcpStringBRTrackingServer()\"And many more, please see the examples folder."
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
    "location": "index.html#Contributors-1",
    "page": "Home",
    "title": "Contributors",
    "category": "section",
    "text": "Authors directly involved with this package are:D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. LeonardWe are grateful for many, many contributions within the Julia package ecosystem – see the REQUIRE files of Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs and others for a far reaching list of contributions."
},

{
    "location": "index.html#Cite-1",
    "page": "Home",
    "title": "Cite",
    "category": "section",
    "text": "Consider citing our work:@misc{caesarjl,\n  author = \"Dehann Fourie, John Leonard, Micheal Kaess, and contributors\",\n  title =  \"Caesar.jl\",\n  year =   2017,\n  url =    \"https://github.com/dehann/Caesar.jl\"\n}"
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
    "location": "getting_started.html#",
    "page": "Getting Started",
    "title": "Getting Started",
    "category": "page",
    "text": ""
},

{
    "location": "getting_started.html#Getting-Started-1",
    "page": "Getting Started",
    "title": "Getting Started",
    "category": "section",
    "text": "Caesar.jl is one of the packages within the JuliaRobotics community, and adheres to the code-of-conduct."
},

{
    "location": "getting_started.html#Local-copy-of-Julia-1",
    "page": "Getting Started",
    "title": "Local copy of Julia",
    "category": "section",
    "text": "Although Julia (or JuliaPro) can be installed on a Linux computer using the apt package manager, we are striving for a fully local installation environment which is highly reproducible on a variety of platforms.The easiest method is–-via the terminal–-to download the desired version of Julia as a binary, extract, setup a symbolic link, and run:cd ~\nmkdir -p julia-software\ncd julia-software\nwget https://julialang-s3.julialang.org/bin/linux/x64/0.6/julia-0.6.4-linux-x86_64.tar.gz\ntar -xvf julia-0.6.4-linux-x86_64\ncd /usr/bin\nsudo ln -s ~/julia-software/julia-9d11f62bcb/bin/julia julia064\nsudo ln -s julia064 juliaNOTE Feel free to modify this setup as you see fit.This should allow any terminal or process on the computer to run the Julia REPL:user@...$ julia\n\n   _       _ _(_)_     |  A fresh approach to technical computing\n  (_)     | (_) (_)    |  Documentation: https://docs.julialang.org\n   _ _   _| |_  __ _   |  Type \"?help\" for help.\n  | | | | | | |/ _` |  |\n  | | |_| | | | (_| |  |  Version 0.6.3 (2018-05-28 20:20 UTC)\n _/ |\\__\'_|_|_|\\__\'_|  |  Official http://julialang.org/ release\n|__/                   |  x86_64-pc-linux-gnu\n\njulia> println(\"hello world\")\nhello worldMaybe a script, or command:user@...$ echo \"println(\\\"hello again\\\")\" > myscript.jl\nuser@...$ julia myscript.jl\nhello again\nuser@...$ rm myscript.jl\n\nuser@...$ julia -e \"println(\\\"one more time.\\\")\"\none more time.NOTE When searching for Julia related help online, use the phrase \'julialang\' instead of just \'julia\'. For example, search for \'julialang workflow tips\' or \'julialang performance tips\'."
},

{
    "location": "getting_started.html#Just-In-Time-Compiling-(i.e.-why-are-first-runs-slow?)-1",
    "page": "Getting Started",
    "title": "Just-In-Time Compiling (i.e. why are first runs slow?)",
    "category": "section",
    "text": "Julia uses just-in-time compilation (unless pre-compiled)  which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use."
},

{
    "location": "getting_started.html#Setup-Juno-IDE-Environment-1",
    "page": "Getting Started",
    "title": "Setup Juno IDE Environment",
    "category": "section",
    "text": "Juno IDE allows for interactive development of Julia code by extending the Atom text editor with a few packages. Download and install Atom as instructed on the website, or via command line:cd ~/Downloads\nwget https://atom.io/download/deb\ndpkg -i atom-amd64.debAfter installing and running Atom, you can choose to either install uber-juno package in one go or install the three associated packages individually. In Atom, open the command pallette by pressing Ctrl + Shft + p and typing settings. Go to the install tab, search for and install these packages:ink\njulia-client\njulia-languageNOTE Some situations have required separately installing the Atom.jl Julia package via command line and Julia REPL; this is done with Julia\'s package manager and Pkg.add(\"Atom\"):$ julia\n...\njulia> Pkg.add(\"Atom\")There are a variety of useful packages in Atom, such as minimap and minimap-git.To install the Julia packages related to Caesar.jl–-which are independent of the Atom packages installed above–-please follow instructions below."
},

{
    "location": "getting_started.html#Julia-Packages-1",
    "page": "Getting Started",
    "title": "Julia Packages",
    "category": "section",
    "text": "The philosophy around Julia packages are discussed at length in the Julia core documentation, where each Julia package relates to a git repository likely found on Github.com. To install a Julia package, simply open a julia REPL (equally the julia REPL in Atom/Juno) and type:Pkg.add(\"Caesar\")These are registered packages maintained by JuliaLang/METADATA.jl. Unregistered packages can also be installed with using only the Pkg.clone function:# Just using Caesar URL as an example --  Caesar is already registered with METADATA\nPkg.clone(\"https://github.com/JuliaRobotics/Caesar.jl.git\")Unless you change the default environment variable JULIA_PKG_DIR, all packages (git repos) are cloned/installed to ~/.julia. You can work with the packages as regular git repositories there."
},

{
    "location": "getting_started.html#Install-Visualization-Utils-(e.g.-Arena.jl)-1",
    "page": "Getting Started",
    "title": "Install Visualization Utils (e.g. Arena.jl)",
    "category": "section",
    "text": "Visualizations were removed from Caesar by popular demand and moved to a new package Arena.jl instead. Please follow instructions on the Visualizations page for a variety of 2D / 3D utilities."
},

{
    "location": "getting_started.html#Contributing,-Issues,-or-Comments-1",
    "page": "Getting Started",
    "title": "Contributing, Issues, or Comments",
    "category": "section",
    "text": "Please feel free to open issues with Caesar.jl or even Fork and Pull Request as required. General conversations or comments can be made in the Caesar Gist."
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
    "text": "#$ Multi-modal range only example:   Click gif below for link to Vimeo, also see tutorial for this example here:<a href=\"http://vimeo.com/190052649\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>"
},

{
    "location": "examples.html#Mutli-session-Use-case-1",
    "page": "Examples",
    "title": "Mutli-session Use-case",
    "category": "section",
    "text": "Multi-session Turtlebot example of the second floor in the Stata Center:   <img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/turtlemultisession.gif\" alt=\"Turtlebot Multi-session animation\" width=\"480\" border=\"0\" /></a>"
},

{
    "location": "examples.html#Simulated-Ambiguous-SONAR-in-3D-1",
    "page": "Examples",
    "title": "Simulated Ambiguous SONAR in 3D",
    "category": "section",
    "text": "Intersection of ambiguous elevation angle from planar SONAR sensor:   <a href=\"http://vimeo.com/198237738\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovasfm02.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>Bi-modal belief   <a href=\"http://vimeo.com/198872855\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovyaw90.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>"
},

{
    "location": "examples.html#More-Examples-1",
    "page": "Examples",
    "title": "More Examples",
    "category": "section",
    "text": "Please see examples folders for Caesar and RoME for more examples, with expanded documentation in the works."
},

{
    "location": "tutorialcontinuousscalar.html#",
    "page": "Entry Tutorial: ContinuousScalar",
    "title": "Entry Tutorial: ContinuousScalar",
    "category": "page",
    "text": ""
},

{
    "location": "tutorialcontinuousscalar.html#Tutorials-1",
    "page": "Entry Tutorial: ContinuousScalar",
    "title": "Tutorials",
    "category": "section",
    "text": ""
},

{
    "location": "tutorialcontinuousscalar.html#IncrementalInference.jl-ContinuousScalar-1",
    "page": "Entry Tutorial: ContinuousScalar",
    "title": "IncrementalInference.jl ContinuousScalar",
    "category": "section",
    "text": "This tutorial illustrates how IncrementalInference enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information. This tutorial is rather abstract and the user is free to imagine any system of relationships, for example a robot driving in a one dimensional world, or a time traveler making uncertain jumps forwards and backwards in time. The tutorial implicitly shows a multi-modal uncertainty introduced and transmitted. The tutorial also illustrates consensus through an additional piece of information, which reduces all stochastic variable marginal beliefs to unimodal only beliefs. The example will also illustrate the use of non-Gaussian beliefs and global inference. The tutorial also shows how to create user defined functions. Lastly, the tutorial demonstrates how automatic initialization of variables works.This tutorial requires IncrementalInference v0.3.0+, RoME v0.1.0, RoMEPlotting packages be installed. In addition, the optional GraphViz package will allow easy visualization of the FactorGraph object structure.To start, the two major mathematical packages are brought into scope.using Distributions\nusing IncrementalInferenceThis tutorial calls for multiple variable nodes connected through algebraic functions stochastic uncertainty. User scope Prior, LinearOffset, and MultiModalOffset with arbitrary distributions are defined as:import IncrementalInference: getSample\n\nstruct Prior{T} <: IncrementalInference.FunctorSingleton where T <: Distribution\n  z::T\nend\ngetSample(s::Prior, N::Int=1) = (rand(s.z,N), )\nstruct LinearOffset{T} <: IncrementalInference.FunctorPairwise where T <: Distribution\n  z::T\nend\ngetSample(s::LinearOffset, N::Int=1) = (rand(s.z,N), )\nfunction (s::LinearOffset)(res::Array{Float64},\n      idx::Int,\n      meas::Tuple{Array{Float64, 1}},\n      X1::Array{Float64,2},\n      X2::Array{Float64,2}  )\n  #\n  res[1] = meas[1][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nend\nstruct MultiModalOffset <: IncrementalInference.FunctorPairwise\n  z::Vector{Distribution}\n  c::Categorical\nend\ngetSample(s::MultiModalOffset, N::Int=1) = (rand.(s.z, N)..., rand(s.c, N))\nfunction (s::MultiModalOffset)(res::Array{Float64},\n      idx::Int,\n      meas::Tuple,\n      X1::Array{Float64,2},\n      X2::Array{Float64,2}  )\n  #\n  res[1] = meas[meas[end][idx]][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nendNotice the residual function relating to the two PairwiseFunctor derived definitions. The one dimensional residual functions, res[1] = measurement - prediction, are used during inference to approximate the convolution of conditional beliefs from the sample approximate marginal beliefs of the connected variables.Guidelines for developing your own functions are discussed here (TBD), and we note that mechanizations and manifolds required for robotic simultaneous localization and mapping (SLAM) has been tightly integrated with the expansion package RoME.jl.The next step is to describe the inference problem with a graphical model of type IncrementalInference.FactorGraph. The first step is to create an empty factor graph object and start populating it with variable nodes. The variable nodes are identified by Symbols, namely :x0, :x1, :x2, :x3.# Start with an empty factor graph\nfg = emptyFactorGraph()\n\n# add the first node\naddNode!(fg, :x0, ContinuousScalar)\n\n# this is unary (prior) factor and does not immediately trigger autoinit of :x0.\naddFactor!(fg, [:x0], Prior(Normal(0,1)))Factor graphs are bipartite graphs with factors that act as mathematical structure between interacting variables. After adding node :x0, a singleton factor of type Prior (which was defined by the user earlier) is \'connected to\' variable node :x0. This unary factor is taken as a Distributions.Normal distribution with zero mean and a standard devitation of 1. GraphViz.jl can be used to visualize the factor graph structure, although the package is not installed by default. Furthermore, the writeGraphPdf member definition is given at the end of this tutorial, which allows the user to store the graph image in graphviz supported image types.Graphs.plot(fg.g)\n# writeGraphPdf(fg, file=\"fgx01.pdf\") # file=\"fgx01.png\"The two node factor graph is shown in the image below.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0.png\" width=\"120\" border=\"0\" />\n</p>Automatic initialization of variables depend on how the factor graph model is constructed. This tutorial demonstrates this behavior by first showing that :x0 is not initialized:@show isInitialized(fg, :x0) # falseWhy is :x0 not initialized? Since no other variable nodes have been \'connected to\' (or depend) on :x0 and future intentions of the user are unknown, the initialization of :x0 is deferred until the latest possible moment. IncrementalInference.jl assumes that the user will generally populate new variable nodes with most of the associated factors before moving to the next variable. By delaying initialization of a new variable (say :x0) until a second newer uninitialized variable (say :x1) depends on :x0, the IncrementalInference algorithms hope to then initialize :x0 with the more information from previous and surrounding variables and factors. Also note that initialization of variables is a local operation based only on the neighboring nodes – global inference will over the entire graph is shows later in this tutorial.By adding :x1 and connecting it through the LinearOffset and Normal distributed factor, the automatic initialization of :x0 is triggered.addNode!(fg, :x1, ContinuousScalar)\n# P(Z | :x1 - :x0 ) where Z ~ Normal(10,1)\naddFactor!(fg, [:x0, :x1], LinearOffset(Normal(10.0,1)))\n@show isInitialized(fg, :x0) # trueNote that the automatic initialization of :x0 is aware that :x1 is not initialized and therefore only used the Prior(Normal(0,1)) unary factor to initialize the marginal belief estimate for :x0. The structure of the graph has now been updated to two variable nodes and two factors.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx01.png\" width=\"240\" border=\"0\" />\n</p>Global inference requires that the entire factor graph be initialized before the numerical belief computation algorithms can be performed. Notice how the new :x1 variable is not yet initialized:@show isInitialized(fg, :x1) # falseThe RoMEPlotting.jl package allows visualization (plotting) of the belief state over any of the variable nodes. Remember the first time executions are slow given required code compilation, and that future versions of these package will use more precompilation to reduce first execution running cost.using RoMEPlotting\n\nplotKDE(fg, :x0)<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0.png\" width=\"360\" border=\"0\" />\n</p>By forcing the initialization of :x1 and plotting its belief estimate,ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1])the predicted influence of the P(Z| X1 - X0) = LinearOffset(Normal(10, 1)) is shown by the red trace.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx01.png\" width=\"360\" border=\"0\" />\n</p>The red trace (predicted belief of :x1) is noting more than the approximated convolution of the current marginal belief of :x0 with the conditional belief described by P(Z | X1 - X0).Another ContinuousScalar variable :x2 is \'connected\' to :x1 through a more complicated MultiModalOffset likelihood function.addNode!(fg, :x2, ContinuousScalar)\nmmo = MultiModalOffset([Rayleigh(3); Uniform(30,55)], Categorical([0.4; 0.6]))\naddFactor!(fg, [:x1, :x2], mmo)<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx012.png\" width=\"360\" border=\"0\" />\n</p>The mmo variable illustrates how a near arbitrary mixture probability distribution can be used as a conditional relationship between variable nodes in the factor graph. In this case, a 40%/60% balance of a Rayleigh and truncated Uniform distribution which acts as a multi-modal conditional belief. Interpret carefully what a conditional belief of this nature actually means.Following the tutorial\'s practical example frameworks (robot navigation or time travel), this multi-modal belief implies that moving from one of the probable locations in :x1 to a location in :x2 by some processes defined by mmo=P(Z | X2, X1) is uncertain to the same 40%/60% ratio. In practical terms, collapsing (through observation of an event) the probabilistic likelihoods of the transition from :x1 to :x2 may result in the :x2 location being at either 15-20, or 40-65-ish units. The predicted belief over :x2 is illustrated by plotting the predicted belief (green trace), after forcing initialization.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2])<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx012.png\" width=\"360\" border=\"0\" />\n</p>Adding one more variable :x3 through another LinearOffset(Normal(-50,1))addNode!(fg, :x3, ContinuousScalar)\naddFactor!(fg, [:x2, :x3], LinearOffset(Normal(-50, 1)))expands the factor graph to to four variables and four factors.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0123.png\" width=\"480\" border=\"0\" />\n</p>This part of the tutorial shows how a unimodal likelihood (conditional belief) can transmit the bimodal belief currently contained in :x2.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2, :x3])Notice the blue trace (:x3) is a shifted and slightly spread out version of the initialized belief on :x2, through the convolution with the conditional belief P(Z | X2, X3).<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0123.png\" width=\"480\" border=\"0\" />\n</p>Global inference over the entire factor graph has still not occurred, and will at this stage produce roughly similar results to the predicted beliefs shown above. Only by introducing more information into the factor graph can inference extract more precise marginal belief estimates for each of the variables. A final piece of information added to this graph is a factor directly relating :x3 with :x0.addFactor!(fg, [:x3, :x0], LinearOffset(Normal(40, 1)))Pay close attention to what this last factor means in terms of the probability density traces shown in the previous figure. The blue trace for :x3 has two major modes, one that overlaps with :x0, :x1 near 0 and a second mode further to the left at -40. The last factor introduces a shift LinearOffset(Normal(40,1)) which essentially aligns the left most mode of :x3 back onto :x0.<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/fgx0123c.png\" width=\"480\" border=\"0\" />\n</p>This last factor forces a mode selection through consensus. By doing global inference, the new information obtained in :x3 will be equally propagated to :x2 where only one of the two modes will remain.Global inference is achieved with local computation using two function calls, as follows.tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# and visualization\nplotKDE(fg, [:x0, :x1, :x2, :x3])The resulting posterior marginal beliefs over all the system variables are:<p align=\"center\">\n<img src=\"assets/tutorials/ContinuousScalar/plx0123infr.png\" width=\"480\" border=\"0\" />\n</p>It is import to note that although this tutorial ends with all marginal beliefs having near Gaussian shape and are unimodal, that the package supports multi-modal belief estimates during both the prediction and global inference processes. In fact, many of the same underlying inference functions are involved with the automatic initialization process and the global multi-modal iSAM inference procedure. This concludes the ContinuousScalar tutorial particular to the IncrementalInference package."
},

{
    "location": "tut_hexagonal2d.html#",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Entry Tutorial: Hexagonal 2D SLAM",
    "category": "page",
    "text": ""
},

{
    "location": "tut_hexagonal2d.html#Tutorial:-Hexagonal-2D-SLAM-Example-(Local-Compute)-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Tutorial: Hexagonal 2D SLAM Example (Local Compute)",
    "category": "section",
    "text": "A simple 2D robot trajectory example is expanded below using techniques developed in simultaneous localization and mapping (SLAM). This example is available as a single script here."
},

{
    "location": "tut_hexagonal2d.html#Creating-the-Factor-Graph-with-Pose2-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Creating the Factor Graph with Pose2",
    "category": "section",
    "text": "The first step is to load the required modules, and in our case we will add a few Julia processes to help with the compute later on.  # add more julia processes\nnprocs() < 4 ? addprocs(4-nprocs()) : nothing\n\n# tell Julia that you want to use these modules/namespaces\nusing RoME, DistributionsAfter loading the RoME and Distributions modules, we construct a local factor graph object in memory:# start with an empty factor graph object\nfg = initfg()\n\n# Add the first pose :x0\naddNode!(fg, :x0, Pose2)\n\n# Add at a fixed location PriorPose2 to pin :x0 to a starting location\naddFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*eye(3))) )A factor graph object fg (of type ::FactorGraph) has been constructed; the first pose :x0 has been added; and a prior factor setting the origin at [0,0,0] over variable node dimensions [x,y,θ] in the world frame. The type Pose2 is used to indicate what variable is stored in the node. Caesar.jl allows a little more freedom in how factor and variable nodes can be connected, while still allowing for type-assertion to occur.NOTE Julia uses just-in-time compilation (unless pre-compiled)  which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use.The next 6 nodes are added with odometry in an counter-clockwise hexagonal manner. Note how variables are denoted with symbols, :x2 == Symbol(\"x2\"):# Drive around in a hexagon\nfor i in 0:5\n  psym = Symbol(\"x$i\")\n  nsym = Symbol(\"x$(i+1)\")\n  addNode!(fg, nsym, Pose2)\n  pp = Pose2Pose2(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))\n  addFactor!(fg, [psym;nsym], pp )\nendAt this point it would be good to see what the factor graph actually looks like:writeGraphPdf(fg)You should see the program evince open with this visual:(Image: exfg2d)"
},

{
    "location": "tut_hexagonal2d.html#Performing-Inference-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Performing Inference",
    "category": "section",
    "text": "Let\'s run the multimodal-incremental smoothing and mapping (mm-iSAM) solver against this fg object:# perform inference, and remember first runs are slower owing to Julia\'s just-in-time compiling\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n# batchSolve!(fg) # coming soonThis will take a couple of seconds (including first time compiling for all Julia processes)."
},

{
    "location": "tut_hexagonal2d.html#Some-Visualization-Plot-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Some Visualization Plot",
    "category": "section",
    "text": "2D plots of the factor graph contents is provided by the RoMEPlotting package. See further discussion on visualizations and packages here.## Inter-operating visualization packages for Caesar/RoME/IncrementalInference exist\nusing RoMEPlotting\n\n# For Juno/Jupyter style use\npl = drawPoses(fg)\n\n# For scripting use-cases you can export the image\nGadfly.draw(Gadfly.PDF(\"/tmp/test.pdf\", 20cm, 10cm),pl)  # or PNG(...)(Image: test)"
},

{
    "location": "tut_hexagonal2d.html#Adding-Landmarks-as-Point2-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Adding Landmarks as Point2",
    "category": "section",
    "text": "Suppose some sensor detected a feature of interest with an associated range and bearing measurement The new variable and measurement can be included into the factor graph as follows:# Add landmarks with Bearing range measurements\naddNode!(fg, :l1, Point2, labels=[\"LANDMARK\"])\np2br = Pose2DPoint2DBearingRange(Normal(0,0.1),Normal(20.0,1.0))\naddFactor!(fg, [:x0; :l1], p2br)\n\n# Initialize :l1 numerical values but do not rerun solver\nensureAllInitialized!(fg)NOTE The default behavior for initialization of variable nodes implies the last variable noded added will not have any numerical values yet, please see ContinuousScalar Tutorial for deeper discussion on automatic initialization (autoinit). A slightly expanded plotting function will draw both poses and landmarks (and currently assumes labels starting with :x and :l respectively) – notice the new landmark bottom right:drawPosesLandms(fg)(Image: test)"
},

{
    "location": "tut_hexagonal2d.html#One-type-of-Loop-Closure-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "One type of Loop-Closure",
    "category": "section",
    "text": "Loop-closures are a major part of SLAM based state estimation. One illustration is to take a second sighting of the same :l1 landmark from the last pose :x6; followed by repeating the inference and re-plotting the result – notice the tighter confidences over all variables:# Add landmarks with Bearing range measurements\np2br2 = Pose2DPoint2DBearingRange(Normal(0,0.1),Normal(20.0,1.0))\naddFactor!(fg, [:x6; :l1], p2br2)\n\n# solve\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# redraw\npl = drawPosesLandms(fg)(Image: test)This concludes the Hexagonal 2D SLAM example."
},

{
    "location": "tut_hexagonal2d.html#Interest:-The-Bayes-(Junction)-tree-1",
    "page": "Entry Tutorial: Hexagonal 2D SLAM",
    "title": "Interest: The Bayes (Junction) tree",
    "category": "section",
    "text": "The Bayes (Junction) tree is used as an acyclic (has no loops) computational object, an exact algebraic refactorizating of factor graph, to perform the associated sum-product inference. The visual structure of the tree can extracted by modifying the command tree = wipeBuildNewTree!(fg, drawpdf=true) to produce representations such as this in bt.pdf.(Image: exbt2d)"
},

{
    "location": "tut_slamedonut.html#",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Entry Tutorial: Singular Ranges-only SLAM",
    "category": "page",
    "text": ""
},

{
    "location": "tut_slamedonut.html#Tutorial:-Singular-Ranges-only-SLAM-Solution-(i.e.-\"Under-Constrained\")-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Tutorial: Singular Ranges-only SLAM Solution (i.e. \"Under-Constrained\")",
    "category": "section",
    "text": "This tutorial describes a range-only system where there are always more variable dimensions than range measurements made. The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point – other examples show inference results where highly non-Gaussian error distributions are used. The one pre-baked result of this of this singular range-only illustration can be seen in this video:Multi-modal range only example (click here or image for full Vimeo):   <a href=\"http://vimeo.com/190052649\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"640\" border=\"0\" /></a>This example is also available as a script here in RoME.jl."
},

{
    "location": "tut_slamedonut.html#REQUIRES-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "REQUIRES",
    "category": "section",
    "text": "RoME v0.1.5\nRoMEPlotting v0.0.2"
},

{
    "location": "tut_slamedonut.html#Loading-The-Data-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Loading The Data",
    "category": "section",
    "text": "Starting a Juno IDE or Julia REPL session, the ground truth positions for vehicle positions GTp and landmark positions GTl can be loaded into memory directly with these values:GTp = Dict{Symbol, Vector{Float64}}()\nGTp[:l100] = [0.0;0]\nGTp[:l101] = [50.0;0]\nGTp[:l102] = [100.0;0]\nGTp[:l103] = [100.0;50.0]\nGTp[:l104] = [100.0;100.0]\nGTp[:l105] = [50.0;100.0]\nGTp[:l106] = [0.0;100.0]\nGTp[:l107] = [0.0;50.0]\nGTp[:l108] = [0.0;-50.0]\nGTp[:l109] = [0.0;-100.0]\nGTp[:l110] = [50.0;-100.0]\nGTp[:l111] = [100.0;-100.0]\nGTp[:l112] = [100.0;-50.0]\n\nGTl = Dict{Symbol, Vector{Float64}}()\nGTl[:l1] = [10.0;30]\nGTl[:l2] = [30.0;-30]\nGTl[:l3] = [80.0;40]\nGTl[:l4] = [120.0;-50]NOTE 1. that by using location indicators :l1, :l2, ... or :l100, :l101, ... is of practical benefit when visualizing with existing RoMEPlotting functions.NOTE 2. Landmarks must be in range before range measurements can be made to them."
},

{
    "location": "tut_slamedonut.html#Creating-the-Factor-Graph-with-Point2-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Creating the Factor Graph with Point2",
    "category": "section",
    "text": "The first step is to load the required modules, and in our case we will add a few Julia processes to help with the compute later on.  # add more julia processes\nnprocs() < 7 ? addprocs(7-nprocs()) : nothing\n\n# tell Julia that you want to use these modules/namespaces\nusing RoME, DistributionsNOTE Julia uses just-in-time compiling (unless pre-compiled), therefore each time a new function call on a Julia process will be slow, but all following calls to the same functions will be as fast as the statically compiled code.This example exclusively uses Point2 variable node types, which have dimension 2 and represent [x, y] position estimates in the world frame.Next construct the factor graph containing the first pose :l100 (without any knowledge of where it is) and three measured beacons/landmarks :l1,:l2,:l3 – with prior location knowledge for :l1 and :l2:# create the factor graph object\nfg = initfg()\n\n# first pose with no initial estimate\naddNode!(fg, :l100, Point2)\n\n# add three landmarks\naddNode!(fg, :l1, Point2)\naddNode!(fg, :l2, Point2)\naddNode!(fg, :l3, Point2)\n\n# and put priors on :l101 and :l102\naddFactor!(fg, [:l1;], PriorPoint2(MvNormal(GTl[:l1], eye(2))) )\naddFactor!(fg, [:l2;], PriorPoint2(MvNormal(GTl[:l2], eye(2))) )The PriorPoint2 is assumed to be a multivariate normal distribution of covariance eye(2), as well as a weighting factor of [1.0].NOTE API change: PriorPoint2D changed to PriorPoint2{T} to accept distribution objects and discard (standard in RoME v0.1.5 – see issue 72 here)."
},

{
    "location": "tut_slamedonut.html#Adding-Range-Measurements-Between-Variables-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Adding Range Measurements Between Variables",
    "category": "section",
    "text": "Next we connect the three range measurements from the vehicle location :l0 to the three beacons, respectively – and consider that the range measurements are completely relative between the vehicle and beacon position estimates:# first range measurement\nrhoZ1 = norm(GTl[:l1]-GTp[:l100])\nppr = Point2DPoint2DRange([rhoZ1], 2.0, [1.0])\naddFactor!(fg, [:l100;:l101], ppr)\n\n# second range measurement\nrhoZ2 = norm(GTl[:l2]-GTp[:l100])\nppr = Point2DPoint2DRange([rhoZ2], 3.0, [1.0])\naddFactor!(fg, [:l100; :l2], ppr)\n\n# second range measurement\nrhoZ3 = norm(GTl[:l3]-GTp[:l100])\nppr = Point2DPoint2DRange([rhoZ3], 3.0, [1.0])\naddFactor!(fg, [:l100; :l3], ppr)The ranging measurement standard deviation of 2.0 or 3.0 is taken, assuming a Gaussian measurement assumption.   Again, any distribution could have been used. The factor graph should look as follows:writeGraphPdf(fg) # show the factor graph(Image: exranges01)"
},

{
    "location": "tut_slamedonut.html#Inference-and-Visualizations-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Inference and Visualizations",
    "category": "section",
    "text": "At this point we can call the solver start interpreting the first results:tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)The factor graph figure above showed the structure between variables and factors. In order to see the numerical values contained in the factor graph, a set of tools are provided by the RoMEPlotting and KernelDensityEstimatePlotting packages. For more details, please see the dedicated visualization discussion here.First look at the two landmark positions :l1, :l2 at (10.0,30),(30.0,-30) respectively.using RoMEPlotting\n\nplotKDE(fg, [:l1;:l2], dims=[1;2], levels=4)(Image: testl1_2)Similarly, the belief estimate for the first vehicle position :l100 is bi-modal, due to the intersection of two range measurements:plotKDE(fg, :l100, dims=[1;2])(Image: testl100)An alternative plotting interface can also be used, that shows a histogram of desired elements instead:drawLandms(fg, from=1, to=101)(Image: testlall)Notice the ring of particles which represents the belief on the third beacon/landmark :l3, which was not constrained by a prior factor. Instead, the belief over the position of :l3 is being estimated simultaneous to estimating the vehicle position :l100."
},

{
    "location": "tut_slamedonut.html#Stochastic-Growth-and-Decay-of-Modes-(i.e.-Hypotheses)-1",
    "page": "Entry Tutorial: Singular Ranges-only SLAM",
    "title": "Stochastic Growth and Decay of Modes (i.e. Hypotheses)",
    "category": "section",
    "text": "Next consider the vehicle moving a distance of 50 units–-and by design the direction of travel is not known–-to the next true position. The video above gives away the vehicle position with the cyan line, showing travel in the shape of a lower case \'e\'. Finally, to speed things up, lets write a function that handles the travel (pseudo odometry factors between positions) and ranging measurement factors to beacons.function vehicle_drives_to!(fgl::FactorGraph, pos_sym::Symbol, GTp::Dict, GTl::Dict; measurelimit::R=150.0) where {R <: Real}\n  currvar = union(ls(fgl)...)\n  prev_sym = Symbol(\"l$(maximum(Int[parse(Int,string(currvar[i])[2:end]) for i in 2:length(currvar)]))\")\n  if !(pos_sym in currvar)\n    println(\"Adding variable vertex $pos_sym, not yet in fgl::FactorGraph.\")\n    addNode!(fgl, pos_sym, Point2)\n    @show rho = norm(GTp[prev_sym] - GTp[pos_sym])\n    ppr = Point2DPoint2DRange([rho], 3.0, [1.0])\n    addFactor!(fgl, [prev_sym;pos_sym], ppr)\n  else\n    warn(\"Variable node $pos_sym already in the factor graph.\")\n  end\n  beacons = keys(GTl)\n  for ll in beacons\n    rho = norm(GTl[ll] - GTp[pos_sym])\n    # Check for feasible measurements:  vehicle within 150 units from the beacons/landmarks\n    if rho < measurelimit\n      ppr = Point2DPoint2DRange([rho], 3.0, [1.0])\n      if !(ll in currvar)\n        println(\"Adding variable vertex $ll, not yet in fgl::FactorGraph.\")\n        addNode!(fgl, ll, Point2)\n      end\n      addFactor!(fgl, [pos_sym;ll], ppr)\n    end\n  end\n  nothing\nendAfter pasting (or running) this function in the Julia, a new member definition exists for vehicle_drives_to!.NOTE The exclamation mark at the end of the function name has no syntactic significance in Julia, since the full UTF8 character set is available for functions or variables. Instead, the exclamation serves as a Julia community convention to tell the caller that this function will modify the contents of at least some of the variables being passed into it – in this case the factor graph fg will be modified.Now the actual driving event can be added to the factor graph:#drive to location :l101, then :l102\nvehicle_drives_to!(fg, :l101, GTp, GTl)\nvehicle_drives_to!(fg, :l102, GTp, GTl)\n\n# see the graph\nwriteGraphPdf(fg)NOTE The distance traveled could be any combination of accrued direction and speeds, however, a straight line Gaussian error model is used to keep the visual presentation of this example as simple as possible.The marginal posterior estimates are found by repeating inference over the factor graph, followed drawing all vehicle locations as a contour map:tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# draw all vehicle locations\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 0:2], dims=[1;2])\n# Gadfly.draw(PDF(\"/tmp/testL100_102.pdf\", 20cm, 10cm),pl) # for storing image to disk\n\npl = plotKDE(fg, [:l3;:l4], dims=[1;2], levels=4)\n# Gadfly.draw(PNG(\"/tmp/testL3_4.png\", 20cm, 10cm),pl)Notice how the vehicle positions have two hypotheses, one left to right and one diagonal right to bottom left – both are valid solutions!(Image: testl100_102)The two \"free\" beacons/landmarks :l3,:l4 still have several modes each, implying insufficient data to constrain either to a strong unimodal belief.(Image: testl3_4)\nvehicle_drives_to!(fg, :l103, GTp, GTl)\nvehicle_drives_to!(fg, :l104, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 0:4], dims=[1;2])\n# Gadfly.draw(PDF(\"/tmp/testL100_104.pdf\", 20cm, 10cm),pl)Moving up to position :l104 still shows strong multiodality in the vehicle position estimates:(Image: testl100_105)vehicle_drives_to!(fg, :l105, GTp, GTl)\nvehicle_drives_to!(fg, :l106, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\nvehicle_drives_to!(fg, :l107, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\nvehicle_drives_to!(fg, :l108, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 2:8], dims=[1;2], levels=6)\n# Gadfly.draw(PDF(\"/tmp/testL103_108.pdf\", 20cm, 10cm),pl)Next we see a strong return to a single dominant mode in all vehicle position estimates, owing to the increased measurements to beacons/landmarks as well as more unimodal estimates in :l3, :l4 beacon/landmark positions.vehicle_drives_to!(fg, :l109, GTp, GTl)\nvehicle_drives_to!(fg, :l110, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\nvehicle_drives_to!(fg, :l111, GTp, GTl)\nvehicle_drives_to!(fg, :l112, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree, N=200)\n\n\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 7:12], dims=[1;2])\n# Gadfly.draw(PDF(\"/tmp/testL106_112.pdf\", 20cm, 10cm),pl)\n\npl = plotKDE(fg, [:l1;:l2;:l3;:l4], dims=[1;2], levels=4)\n# Gadfly.draw(PDF(\"/tmp/testL1234.pdf\", 20cm, 10cm),pl)\n\npl = drawLandms(fg, from=100)\n# Gadfly.draw(PDF(\"/tmp/testLocsAll.pdf\", 20cm, 10cm),pl)Several location belief estimates exhibit multimodality as the trajectory progresses (not shown), but collapses and finally collapses to a stable set of dominant position estimates.(Image: testl106_112)Landmark estimates are also stable at one estimate:(Image: testl1234)In addition, the SLAM 2D landmark visualization can be re-used to plot more information at once:# pl = drawLandms(fg, from=100, to=200)\n# Gadfly.draw(PDF(\"/tmp/testLocsAll.pdf\", 20cm, 10cm),pl)\n\npl = drawLandms(fg)\n# Gadfly.draw(PDF(\"/tmp/testAll.pdf\", 20cm, 10cm),pl)(Image: testall)This example used the default of N=200 particles per marginal belief. By increasing the number to N=300 throughout the test many more modes and interesting features can be explored, and we refer the reader to an alternative and longer discussion on the same example, in Chapter 6 here."
},

{
    "location": "definingfactors.html#",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "Moderate Tutorial: Defining Factors",
    "category": "page",
    "text": ""
},

{
    "location": "definingfactors.html#IncrementalInference.jl-Defining-Factors-(2017-API)-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "IncrementalInference.jl Defining Factors (2017 API)",
    "category": "section",
    "text": "This tutorial describes how a new factor can be developed, beyond the pre-existing implementation in RoME.jl.  Factors can accept any number of variable dependencies and allow for a wide class of allowable function calls can be used.  Our intention is to make it as easy as possible for users to create their own factor types.A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models – for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion here (author disclosure): (Image: factorgraphexample)"
},

{
    "location": "definingfactors.html#Example:-Adding-Velocity-to-Point2D-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "Example: Adding Velocity to Point2D",
    "category": "section",
    "text": "A smaller example in two dimensions where we wish to estimate the velocity of some target:  Consider two variables :x0 with a prior as well as a conditional–-likelihood for short–-to variable :x1.  Priors are in the \"global\" reference frame (how ever you choose to define it), while likelihoods are in the \"local\" / \"relative\" frame that only exist between variables.(Image: dynpoint2fg)"
},

{
    "location": "definingfactors.html#Brief-on-Variable-Node-softtypes-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "Brief on Variable Node softtypes",
    "category": "section",
    "text": "Variable nodes retain meta data (so called \"soft types\") describing the type of variable.  Common VariableNode types are RoME.Point2D, RoME.Pose3D.  VariableNode soft types are passed during construction of the factor graph, for example:v1 = addNode!(fg, :x1, Pose2)Certain cases require that more information be retained for each VariableNode, and velocity calculations are a clear example where time stamp data across positions is required.  Note Larger data can also be stored under the bigdata framework which is discussed here (TBD).If the required VariableNode does not exist, then one can be created, such as adding velocity states with DynPoint2:mutable struct DynPoint2 <: IncrementalInference.InferenceVariable\n  ut::Int64 # microsecond time\n  dims::Int\n  labels::Vector{String}\n  DynPoint2(;ut::Int64=0, labels::Vector{<:AbstractString}=String[]) = new(ut, 4, labels)\nendThe dims field is permanently set to 4, i.e. [x, y, dx/dt, dy/dt].  Labels represent special labels that can be used for more efficient indexing in database systems.  The utparameter is for storing the microsecond time stamp for that variable node.In order to implement your own factor type outside IncrementalInference you should import the required identifiers, as follows:using IncrementalInference\nimport IncrementalInference: getSampleNote that new factor types can be defined at any time, even after you have started to construct the FactorGraph object."
},

{
    "location": "definingfactors.html#DynPoint2VelocityPrior-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "DynPoint2VelocityPrior",
    "category": "section",
    "text": "Work in progress.mutable struct DynPoint2VelocityPrior{T} <: IncrementalInference.FunctorSingleton where {T <: Distribution}\n  z::T\n  DynPoint2VelocityPrior{T}() where {T <: Distribution} = new{T}()\n  DynPoint2VelocityPrior(z1::T) where {T <: Distribution} = new{T}(z1)\nend\ngetSample(dp2v::DynPoint2VelocityPrior, N::Int=1) = (rand(dp2v.z,N), )"
},

{
    "location": "definingfactors.html#DynPoint2DynPoint2-(preintegration)-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "DynPoint2DynPoint2 (preintegration)",
    "category": "section",
    "text": "The basic idea is that change in position is composed of three components (originating from double integration of Newton\'s second law):(Image: deltapositionplus) ( eq. 1)DynPoint2DynPoint2 factor is using the above equation to define the difference in position between the two DynPoint2s.  The position part stored in DynPoint2DynPoint2 factor corresponds to (Image: deltaposplusonly).  A new multi-variable (so called \"pairwise\") factor between any number of variables is defined with three elements:Factor type definition that inherits either IncrementalInference.FunctorPairwise or IncrementalInference.FunctorPairwiseMinimize;mutable struct DynPoint2DynPoint2{T} <: IncrementalInference.FunctorPairwise where {T <: Distribution}\n  z::T\n  DynPoint2DynPoint2{T}() where {T <: Distribution} = new{T}()\n  DynPoint2DynPoint2(z1::T) where {T <: Distribution} = new{T}(z1)\nendA sampling function with exactly the signature: getSample(dp2dp2::DynPoint2DynPoint2, N::Int=1) and returning a Tuple (legacy reasons);getSample(dp2dp2::DynPoint2DynPoint2, N::Int=1) = (rand(dp2dp2.z,N), )A residual or minimization function with exactly the signature described below.Residual (related to FunctorPairwise) or factor minimization function (related to FunctorPairwiseMinimize) signatures should match this dp2dp2::DynPoint2DynPoint2 example:function (dp2dp2::DynPoint2DynPoint2)(\n            res::Array{Float64},\n            userdata,\n            idx::Int,\n            meas::Tuple,\n            Xs...  )::Voidwhere Xs can be expanded to the particular number of variable nodes this factor will be associated, and note they are order sensitive at addFactor!(fg, ...) time.  The res parameter is a vector of the same dimension defined by the largest of the Xs terms.  The userdata value contains the small metadata / userdata portions of information that was introduced to the factor graph at construction time – please consult error(string(fieldnames(userdata))) for details at this time.  This is a relatively new feature in the code and likely to be improved.  The idx parameter represents a legacy index into the measurement meas[1] and variables Xs to select the desired marginal sample value.  Future versions of the code plan to remove the idx parameter entirely.  The Xs array of parameter are each of type ::Array{Float64,2} and contain the estimated samples from each of the current best marginal belief estimates of the factor graph variable node.  function (dp2dp2::DynPoint2DynPoint2)(\n            res::Array{Float64},\n            userdata,\n            idx::Int,\n            meas::Tuple,\n            Xi::Array{Float64,2},\n            Xj::Array{Float64,2}  )\n  #\n  z = meas[1][:,idx]\n  xi, xj = Xi[:,idx], Xj[:,idx]\n  dt = (userdata.variableuserdata[2].ut - userdata.variableuserdata[1].ut)*1e-6   # roughly the intended use of userdata\n  res[1:2] = z[1:2] - (xj[1:2] - (xi[1:2]+dt*xi[3:4]))\n  res[3:4] = z[3:4] - (xj[3:4] - xi[3:4])\n  nothing\nendA brief usage example looks as follows, and further questions about how the preintegration strategy was implemented can be traced through the original issue JuliaRobotics/RoME.jl#60 or the literature associated with this project, or contact for more information.using RoME, Distributions\nfg = initfg()\nv0 = addNode!(fg, :x0, DynPoint2(ut=0))\n\n# Prior factor as boundary condition\npp0 = DynPoint2VelocityPrior(MvNormal([zeros(2);10*ones(2)], 0.1*eye(4)))\nf0 = addFactor!(fg, [:x0;], pp0)\n\n# conditional likelihood between Dynamic Point2\nv1 = addNode!(fg, :x1, DynPoint2(ut=1000_000)) # time in microseconds\ndp2dp2 = DynPoint2DynPoint2(MvNormal([10*ones(2);zeros(2)], 0.1*eye(4)))\nf1 = addFactor!(fg, [:x0;:x1], dp2dp2)\n\nensureAllInitialized!(fg)\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\nusing KernelDensityEstimate\n@show x0 = getKDEMax(getVertKDE(fg, :x0))\n# julia> ... = [-0.19441, 0.0187019, 10.0082, 10.0901]\n@show x1 = getKDEMax(getVertKDE(fg, :x1))\n # julia> ... = [19.9072, 19.9765, 10.0418, 10.0797]"
},

{
    "location": "definingfactors.html#VelPoint2VelPoint2-(back-differentiation)-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "VelPoint2VelPoint2 (back-differentiation)",
    "category": "section",
    "text": "In case the preintegrated approach is not the first choice, we include VelPoint2VelPoint2 <: IncrementalInference.FunctorPairwiseMinimize as a second likelihood factor example which may seem more intuitive:mutable struct VelPoint2VelPoint2{T} <: IncrementalInference.FunctorPairwiseMinimize where {T <: Distribution}\n  z::T\n  VelPoint2VelPoint2{T}() where {T <: Distribution} = new{T}()\n  VelPoint2VelPoint2(z1::T) where {T <: Distribution} = new{T}(z1)\nend\ngetSample(vp2vp2::VelPoint2VelPoint2, N::Int=1) = (rand(vp2vp2.z,N), )\nfunction (vp2vp2::VelPoint2VelPoint2)(\n                res::Array{Float64},\n                userdata,\n                idx::Int,\n                meas::Tuple,\n                Xi::Array{Float64,2},\n                Xj::Array{Float64,2}  )\n  #\n  z = meas[1][:,idx]\n  xi, xj = Xi[:,idx], Xj[:,idx]\n  dt = (userdata.variableuserdata[2].ut - userdata.variableuserdata[1].ut)*1e-6   # roughly the intended use of userdata\n  dp = (xj[1:2]-xi[1:2])\n  dv = (xj[3:4]-xi[3:4])\n  res[1] = 0.0\n  res[1] += sum((z[1:2] - dp).^2)\n  res[1] += sum((z[3:4] - dv).^2)\n  res[1] += sum((dp/dt - xi[3:4]).^2)  # (dp/dt - 0.5*(xj[3:4]+xi[3:4])) # midpoint integration\n  res[1]\nendA similar usage example here shows:fg = initfg()\n\n# add three point locations\nv0 = addNode!(fg, :x0, DynPoint2(ut=0))\nv1 = addNode!(fg, :x1, DynPoint2(ut=1000_000))\nv2 = addNode!(fg, :x2, DynPoint2(ut=2000_000))\n\n# Prior factor as boundary condition\npp0 = DynPoint2VelocityPrior(MvNormal([zeros(2);10*ones(2)], 0.1*eye(4)))\nf0 = addFactor!(fg, [:x0;], pp0)\n\n# conditional likelihood between Dynamic Point2\ndp2dp2 = VelPoint2VelPoint2(MvNormal([10*ones(2);zeros(2)], 0.1*eye(4)))\nf1 = addFactor!(fg, [:x0;:x1], dp2dp2)\n\n# conditional likelihood between Dynamic Point2\ndp2dp2 = VelPoint2VelPoint2(MvNormal([10*ones(2);zeros(2)], 0.1*eye(4)))\nf2 = addFactor!(fg, [:x1;:x2], dp2dp2)\n\n# Graphs.plot(fg.g)\nensureAllInitialized!(fg)\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# see the output\n@show x0 = getKDEMax(getVertKDE(fg, :x0))\n@show x1 = getKDEMax(getVertKDE(fg, :x1))\n@show x2 = getKDEMax(getVertKDE(fg, :x2))Producing output:x0 = getKDEMax(getVertKDE(fg, :x0)) = [0.101503, -0.0273216, 9.86718, 9.91146]\nx1 = getKDEMax(getVertKDE(fg, :x1)) = [10.0087, 9.95139, 10.0622, 10.0195]\nx2 = getKDEMax(getVertKDE(fg, :x2)) = [19.9381, 19.9791, 10.0056, 9.92442]"
},

{
    "location": "definingfactors.html#IncrementalInference.jl-Defining-Factors-(Future-API)-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "IncrementalInference.jl Defining Factors (Future API)",
    "category": "section",
    "text": "We would like to remove the idx indexing from the residual function calls, since that is an unnecessary burden on the user.  Instead, the package will use views and SubArray types to simplify the interface.  Please contact author for more details (8 June 2018)."
},

{
    "location": "definingfactors.html#Contributions-1",
    "page": "Moderate Tutorial: Defining Factors",
    "title": "Contributions",
    "category": "section",
    "text": "Thanks to mc2922 for raising the catalyst issue and conversations that followed from JuliaRobotics/RoME.jl#60."
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
    "text": "Caesar.jl uses the Arena.jl package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl. Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.Over time, Caesar.jl has used a least three different 3D visualization technologies, with the most recent based on WebGL and three.js by means of the MeshCat.jl package. The previous incarnation used a client side installation of VTK  by means of the DrakeVisualizer.jl and Director libraries. Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base. Each epoch has aimed at reducing dependencies and increasing multi-platform support.The sections below discuss 2D and 3D visualization techniques available to the Caesar.jl robot navigation system. Visualization examples will be seen throughout the Caesar.jl package documentation.Note that all visualizations used to be part of the Caesar.jl package itself, but was separated out to Arena.jl in early 2018."
},

{
    "location": "arena_visualizations.html#Installation-1",
    "page": "Arena Visualization",
    "title": "Installation",
    "category": "section",
    "text": "The current version of Arena has a rather large VTK dependency (which compile just fine on Ubuntu/Debian, or maybe even MacOS) wrapped in the DrakeVisualizer.jl package.  This requires the following preinstalled packages:    sudo apt-get install libvtk5-qt4-dev python-vtkNOTE Smaller individual 2D packages can be installed instead – i.e.:Pkg.add(\"RoMEPlotting\")For the full 2D/3D visualization tools used by Caesar.jl–-in a Julia or (JuliaPro) terminal/REPL–-type:julia> Pkg.add(\"Arena\")NOTE Current development will allow the user to choose a three.js WebGL based viewer instead MeshCat.jl."
},

{
    "location": "arena_visualizations.html#D-Visualization-1",
    "page": "Arena Visualization",
    "title": "2D Visualization",
    "category": "section",
    "text": "2D plot visualizations, provided by RoMEPlotting.jl and KernelDensityEstimatePlotting.jl, are generally useful for repeated analysis of a algorithm or data set being studied. These visualizations are often manipulated to emphasize particular aspects of mobile platform navigation. Arena.jl is intended to simplify the process 2D plotting for robot trajectories in two or three dimensions. The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets."
},

{
    "location": "arena_visualizations.html#Hexagonal-2D-SLAM-example-visualization-1",
    "page": "Arena Visualization",
    "title": "Hexagonal 2D SLAM example visualization",
    "category": "section",
    "text": "The major 2D plotting functions between RoMEPlotting.jl:drawPoses\ndrawPosesLandms\ndrawSubmapsand KernelDensityEstimatePlotting.jl:plotKDE / plot(::KernelDensityEstimate)This simplest example for visualizing a 2D robot trajectory–-such as first running the Hexagonal 2D SLAM example–-# Assuming some fg::FactorGraph has been loaded/constructed\n# ...\n\nusing RoMEPlotting\n\n# For Juno/Jupyter style use\npl = drawPosesLandms(fg)\n\n# For scripting use-cases you can export the image\nGadfly.draw(PDF(\"/tmp/test.pdf\", 20cm, 10cm),pl)  # or PNG(...)(Image: test)"
},

{
    "location": "arena_visualizations.html#Density-Contour-Map-1",
    "page": "Arena Visualization",
    "title": "Density Contour Map",
    "category": "section",
    "text": "KernelDensityEstimatePlotting (as used in RoMEPlotting) provides an interface to visualize belief densities as counter plots. The following basic example shows some of features of the API, where plotKDE(..., dims=[1;2]) implies the marginal over variables (x,y):using RoME, Distributions\nusing RoMEPlotting\n\nfg = initfg()\naddNode!(fg, :x0, Pose2)\naddFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), eye(3))))\naddNode!(fg, :x1, Pose2)\naddFactor!(fg, [:x0;:x1], Pose2Pose2(MvNormal([10.0;0;0], eye(3))))\n\nensureAllInitialized!(fg)\n\n# plot one contour density\nplX0 = plotKDE(fg, :x0, dims=[1;2])\n# using Gadfly; Gadfly.draw(PNG(\"/tmp/testX0.png\",20cm,10cm),plX0)(Image: test)The contour density relates to the distribution of marginal samples as seen with this Gadfly.jl package histogram comparison.pl1 = drawPoses(fg, to=0);\nX0 = getVal(fg, :x0);\npl2 = Gadfly.plot(x=X0[1,:],y=X0[2,:], Geom.hexbin);\nplH = hstack(pl1, pl2)\n# Gadfly.draw(PNG(\"/tmp/testH.png\",20cm,10cm),plH)(Image: testh)NOTE Red and Green lines represent Port and Starboard direction of Pose2, respectively.Multiple beliefs can be plotted at the same time, while setting levels=4 rather than the default value:plX1 = plotKDE(fg, [:x0; :x1], dims=[1;2], levels=4)\n# Gadfly.draw(PNG(\"/tmp/testX1.png\",20cm,10cm),plX1)(Image: testx1)One dimensional (such as Θ) or a stack of all plane projections is also available:plTh = plotKDE(fg, [:x0; :x1], dims=[3], levels=4)\n# Gadfly.draw(PNG(\"/tmp/testTh.png\",20cm,10cm),plTh)(Image: testth)plAll = plotKDE(fg, [:x0; :x1], levels=3)\n# Gadfly.draw(PNG(\"/tmp/testX1.png\",20cm,15cm),plAll)(Image: testall)NOTE The functions hstack and vstack is provided through the Gadfly package and allows the user to build a near arbitrary composition of plots.Please see KernelDensityEstimatePlotting package source for more features."
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
    "page": "Offloading to Server",
    "title": "Offloading to Server",
    "category": "page",
    "text": ""
},

{
    "location": "database_interactions.html#Common-Data-Persistence-and-Inference-1",
    "page": "Offloading to Server",
    "title": "Common Data Persistence and Inference",
    "category": "section",
    "text": "Coming soon!"
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
    "text": "NOTE WORK IN PROGRESSPages = [\n    \"func_ref.md\"\n]\nDepth = 3"
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
