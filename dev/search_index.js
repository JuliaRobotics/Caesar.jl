var documenterSearchIndex = {"docs": [

{
    "location": "#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": "<p align=\"center\">\n<img src=\"https://user-images.githubusercontent.com/6412556/47298402-ace95500-d5e5-11e8-8607-593525445d14.png\" width=\"480\" border=\"0\" />\n</p>"
},

{
    "location": "#Introduction-1",
    "page": "Home",
    "title": "Introduction",
    "category": "section",
    "text": "Caesar is a modern robotic framework for localization and mapping, reducing the barrier of entry for Simultaneous Localization and Mapping (SLAM). Caesar attempts to address a number of issues that arise in normal SLAM solutions - solving under-defined systems, inference with non-Gaussian measurement distributions, simplifying factor creation, and centralizing factor-graph persistence with databases. Caesar started as part of the thesis \"Multi-modal and Inertial Sensor Solutions for Navigation-type Factor Graphs\" [1]."
},

{
    "location": "#Features-1",
    "page": "Home",
    "title": "Features",
    "category": "section",
    "text": "The Caesar framework has the following features:Factor-graph representation of pose and sensor data\nLocalization using Multi-modal iSAM\nMulti-core inference supporting Pose2, Pose3, Point2, Point3, Multi-modal (multi-hypothesis), IMU preintegration, KDE density, intensity map, partial constraints, null hypothesis, etc\nMulti-modal and non-parametric representation of constraints\nGaussian distributions are but one of the many representations of measurement error\nSimple, extensible framework for creation of new factor types\nMulti-hypothesis representation in the factor-graph\nLocal in-memory solving on the device as well as database-driven centralized solving\nFixed-lag, continuous operation as well as off-line batch solving"
},

{
    "location": "#TLDR-Installation-1",
    "page": "Home",
    "title": "TLDR Installation",
    "category": "section",
    "text": "If you want to skip ahead and add Caesar to your Julia packages, you can install the metadata registered package \'Caesar\'.Caesar can be installed for latest Julia 0.7/1.0 with:julia> ] # to enable package manager\n(v1.0) pkg> add CaesarUnit tests can further be performed for the upstream packages as follows – NOTE first time runs are slow since each new function call or package must first be precompiled.(v1.0) pkg> test IncrementalInference\n...\n(v1.0) pkg> test RoME\n...\n(v1.0) pkg> test Caesar\n..."
},

{
    "location": "#Caesar-Framework-1",
    "page": "Home",
    "title": "Caesar Framework",
    "category": "section",
    "text": ""
},

{
    "location": "#Caesar-Core-1",
    "page": "Home",
    "title": "Caesar Core",
    "category": "section",
    "text": "Caesar is implemented in Julia (and JuliaPro) for a fast, flexible, dynamic and productive robot designer experience. This framework maintains good interoperability with other languages as listed in features below.Caesar.jl is the umbrella repo that depends on RoME.jl and others to support that \'passes through\' the same functionality while introducing more. For example, interaction with database server systems, LCMCore.jl, (future ROS support), and more."
},

{
    "location": "#Caesar-Core-Packages-1",
    "page": "Home",
    "title": "Caesar Core Packages",
    "category": "section",
    "text": "Critically, this package can operate in the conventional SLAM manner, using local dictionaries, or centralize around the FactorGraph through a graph database using CloudGraphs.jl, as discussed here [2]. A variety of plotting, 3D visualization, serialization, LCM middleware, and analysis tools come standard. Please see internal packages, Robot Motion Estimate RoME.jl and back-end solver IncrementalInference.jl.Details about the accompanying packages:IncrementalInference.jl supplies the algebraic logic for factor graph inference with Bayes tree and depends on several packages itself.\nRoME.jl introduces nodes and factors that are useful to robotic navigation.\nRoMEPlotting.jl are a set of scripts that provide MATLAB style plotting of factor graph beliefs, mostly supporting 2D visualization with some support for projections of 3D."
},

{
    "location": "#Caesar-Extensions-1",
    "page": "Home",
    "title": "Caesar Extensions",
    "category": "section",
    "text": ""
},

{
    "location": "#Visualization-1",
    "page": "Home",
    "title": "Visualization",
    "category": "section",
    "text": "Caesar visualization (plotting of results, graphs, and data) is provided in the Arena.jl package, which is a collection of 3D visualization tools and also depends on RoMEPlotting.jl for 2D visualizations."
},

{
    "location": "#Caesar-SDKs-and-APIs-1",
    "page": "Home",
    "title": "Caesar SDKs and APIs",
    "category": "section",
    "text": "The Caesar framework is not limited to direct Julia use. The following Github projects provide access to features of Caesar in their language:C/C++:\nGraff Cpp\nCaesar LCM\nPython:\nSynchronySDKContributions are welcome! If you are developing an extension we would like to help, please feel free to contact us (details below)."
},

{
    "location": "#Next-Steps-1",
    "page": "Home",
    "title": "Next Steps",
    "category": "section",
    "text": "For installation steps, examples/tutorials, and concepts please refer to the following pages:Pages = [\n    \"installation_environment.md\"\n    \"concepts/concepts.md\"\n    \"examples/examples.md\"\n    \"func_ref.md\"\n]\nDepth = 3"
},

{
    "location": "#Future-1",
    "page": "Home",
    "title": "Future",
    "category": "section",
    "text": "This package is a work in progress. Please file issues here as needed to help resolve problems for everyone! We are tracking improvements and new endeavors in the Issues section of this repository.In the future, Caesar will likely interact more closely with repos such as:SensorFeatureTracking.jl\nAprilTags.jl\nRecursiveFiltering.jl"
},

{
    "location": "#JuliaRobotics-Code-of-Conduct-1",
    "page": "Home",
    "title": "JuliaRobotics Code of Conduct",
    "category": "section",
    "text": "The Caesar repository is part of the JuliaRobotics organization and adheres to the JuliaRobotics code-of-conduct."
},

{
    "location": "#Contributors-1",
    "page": "Home",
    "title": "Contributors",
    "category": "section",
    "text": "Authors directly involved with this package are:D. Fourie, S. Claassens, P. Vaz Teixeira, N. Rypkema, S. Pillai, R. Mata, M. Kaess, J. LeonardWe are grateful for many, many contributions within the Julia package ecosystem – see the REQUIRE files of Caesar, Arena, RoME, RoMEPlotting, KernelDensityEstimate, IncrementalInference, NLsolve, DrakeVisualizer, Graphs, CloudGraphs and others for a far reaching list of contributions.Consider citing our work:@misc{caesarjl,\n  author = \"Dehann Fourie, Sam Claassens, John Leonard, Micheal Kaess, and contributors\",\n  title =  \"Caesar.jl\",\n  year =   2017,\n  url =    \"https://github.com/JuliaRobotics/Caesar.jl\"\n}"
},

{
    "location": "#References-1",
    "page": "Home",
    "title": "References",
    "category": "section",
    "text": "[1]  Fourie, D.: \"Multi-modal and Inertial Sensor Solutions to Navigation-type Factor Graph\",\n     Ph.D. Thesis, Massachusetts Institute of Technology Electrical Engineering and Computer Science together with Woods Hole Oceanographic Institution Department for Applied Ocean Science and Engineering, September 2017.\n[2]  Fourie, D., Claassens, S., Pillai, S., Mata, R., Leonard, J.: \"SLAMinDB: Centralized graph\n     databases for mobile robotics\" IEEE International Conference on Robotics and Automation (ICRA),\n     Singapore, 2017."
},

{
    "location": "installation_environment/#",
    "page": "Installation",
    "title": "Installation",
    "category": "page",
    "text": ""
},

{
    "location": "installation_environment/#Getting-Started-1",
    "page": "Installation",
    "title": "Getting Started",
    "category": "section",
    "text": "Caesar.jl is one of the packages within the JuliaRobotics community, and adheres to the code-of-conduct."
},

{
    "location": "installation_environment/#Local-Installation-of-Julia-1",
    "page": "Installation",
    "title": "Local Installation of Julia",
    "category": "section",
    "text": "Although Julia (or JuliaPro) can be installed on a Linux computer using the apt package manager, we are striving for a fully local installation environment which is highly reproducible on a variety of platforms.The easiest method is–-via the terminal–-to download the desired version of Julia as a binary, extract, setup a symbolic link, and run:cd ~\nmkdir -p julia-software\ncd julia-software\nwget https://julialang-s3.julialang.org/bin/linux/x64/1.0/julia-1.0.1-linux-x86_64.tar.gz\ntar -xvf julia-1.0.1-linux-x86_64.tar.gz\ncd /usr/bin\nsudo ln -s ~/julia-software/julia-1.0.1/bin/julia juliaNote Feel free to modify this setup as you see fit.This should allow any terminal or process on the computer to run the Julia REPL by type julia and testing with:println(\"hello world\")\n# Should print \"hello world\"Maybe a script, or command:user@...$ echo \"println(\\\"hello again\\\")\" > myscript.jl\nuser@...$ julia myscript.jl\nhello again\nuser@...$ rm myscript.jl\n\nuser@...$ julia -e \"println(\\\"one more time.\\\")\"\none more time.\nuser@...$ julia -e \"println(\\\"...testing...\\\")\"\n...testing...\nNote: When searching for Julia related help online, use the phrase \'julialang\' instead of just \'julia\'.For example, search for \'julialang workflow tips\' or \'julialang performance tips\'."
},

{
    "location": "installation_environment/#Just-In-Time-Compiling-(i.e.-why-are-first-runs-slow?)-1",
    "page": "Installation",
    "title": "Just-In-Time Compiling (i.e. why are first runs slow?)",
    "category": "section",
    "text": "Julia uses just-in-time compilation (unless pre-compiled)  which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use."
},

{
    "location": "installation_environment/#Setup-Juno-IDE-Environment-1",
    "page": "Installation",
    "title": "Setup Juno IDE Environment",
    "category": "section",
    "text": "Juno IDE allows for interactive development of Julia code by extending the Atom text editor with a few packages. Download and install Atom as instructed on the website, or via command line:cd ~/Downloads\nwget https://atom.io/download/deb\ndpkg -i atom-amd64.debAfter installing and running Atom, you can choose to either install uber-juno package in one go or install the three associated packages individually. In Atom, open the command pallette by pressing Ctrl + Shft + p and typing settings. Go to the install tab, search for and install eitheruber-junoor the individual packages directly:ink\njulia-client\njulia-language\nlatex-completionsNote Some situations have required the user separately installing the Atom.jl Julia package via command line (if Juno does not automatically install Atom.jl for you).  Atom.jl can then be installed with Julia\'s package manager and add Atom:] # activate Pkg manager\n(v1.0) pkg> add AtomThere are a variety of useful packages in Atom, such as minimap and minimap-git.To install the Julia packages related to Caesar.jl–-which are independent of the Atom packages installed above–-please follow instructions below."
},

{
    "location": "installation_environment/#Julia-Packages-1",
    "page": "Installation",
    "title": "Julia Packages",
    "category": "section",
    "text": "The philosophy around Julia packages are discussed at length in the Julia core documentation, where each Julia package relates to a git repository likely found on Github.com. To install a Julia package, simply open a julia REPL (equally the julia REPL in Atom/Juno) and type:] # activate Pkg manager\n(v1.0) pkg> add CaesarThese are registered packages maintained by JuliaLang/METADATA.jl. Unregistered latest packages can also be installed with using only the Pkg.develop function:# Just using Caesar URL as an example --  Caesar is already registered with METADATA\nusing Pkg\nPkg.develop(PackageSpec(url=\"https://github.com/JuliaRobotics/Caesar.jl.git\"))Unless you change the default environment variable JULIA_PKG_DIR, all packages (git repos) are cloned/installed to ~/.julia. You can work with the packages as regular git repositories there."
},

{
    "location": "installation_environment/#Install-Visualization-Utils-(e.g.-Arena.jl)-1",
    "page": "Installation",
    "title": "Install Visualization Utils (e.g. Arena.jl)",
    "category": "section",
    "text": "Visualizations were removed from Caesar and moved to a new package Arena.jl instead. Please follow instructions on the Visualizations page for a variety of 2D / 3D utilities."
},

{
    "location": "installation_environment/#Contributing,-Issues,-or-Comments-1",
    "page": "Installation",
    "title": "Contributing, Issues, or Comments",
    "category": "section",
    "text": "Please feel free to open issues with Caesar.jl or even Fork and Pull Request as required. General conversations or comments can be made in the Caesar Gist."
},

{
    "location": "concepts/concepts/#",
    "page": "Caesar Concepts",
    "title": "Caesar Concepts",
    "category": "page",
    "text": ""
},

{
    "location": "concepts/concepts/#Caesar-Concepts-1",
    "page": "Caesar Concepts",
    "title": "Caesar Concepts",
    "category": "section",
    "text": "A factor graph is a bipartite representation where variables (denoted by larger nodes) are interconnected by a set of factors (smaller nodes) that represent some algebraic interaction between the variables.  Factors must adhere to the limits of probabilistic models – for example conditional likelihoods (between multiple variables) or priors (unary to one variable).  A more heterogeneous factor graph example is shown below, and a broader discussion here (author disclosure): (Image: factorgraphexample).This section discusses the various concepts in the Caesar framework."
},

{
    "location": "concepts/concepts/#Getting-Started-with-Caesar-1",
    "page": "Caesar Concepts",
    "title": "Getting Started with Caesar",
    "category": "section",
    "text": "The initial steps in constructing and solving graphs can be found in Building and Solving Graphs.We also recommend reviewing the various examples available in the Examples section."
},

{
    "location": "concepts/concepts/#Visualization-1",
    "page": "Caesar Concepts",
    "title": "Visualization",
    "category": "section",
    "text": "Caesar supports various visualizations and plots by using Arena, RoMEPlotting, and Director. This is discussed in Visualization with Arena.jl and RoMEPlotting.jl"
},

{
    "location": "concepts/concepts/#Extending-Caesar-1",
    "page": "Caesar Concepts",
    "title": "Extending Caesar",
    "category": "section",
    "text": "The variables and factors in Caesar should be sufficient for the majority of robotic applications, however Caesar allows users to extend the framework without changing the core code. This is discussed in Creating New Variables and Factors."
},

{
    "location": "concepts/concepts/#Connectivity-and-Extensibility-1",
    "page": "Caesar Concepts",
    "title": "Connectivity and Extensibility",
    "category": "section",
    "text": "Caesar supports both in-memory solving (really fast, but for moderately-sized graphs) as well as database-driven solving (think massive graphs and multiple sessions). This is still under development/being refactored, and is discussed in Common Data Persistence and Inference.Although Caesar is Julia-based, it provides multi-language support with a ZMQ interface. This is discussed in Caesar Multi-Language Support."
},

{
    "location": "concepts/building_graphs/#",
    "page": "Building Factor Graphs",
    "title": "Building Factor Graphs",
    "category": "page",
    "text": ""
},

{
    "location": "concepts/building_graphs/#Building-and-Solving-Graphs-1",
    "page": "Building Factor Graphs",
    "title": "Building and Solving Graphs",
    "category": "section",
    "text": "Irrespective of your application - real-time robotics, batch processing of survey data, or really complex multi-hypothesis modeling - you\'re going to need to add factors and variables to a graph. This section discusses how to do that in Caesar.The following sections discuss the steps required to construct a graph and solve it:Initialing the Factor Graph\nAdding Variables and Factors to the Graph\nSolving the Graph\nInforming the Solver About Ready Data"
},

{
    "location": "concepts/building_graphs/#Initializing-a-Factor-Graph-1",
    "page": "Building Factor Graphs",
    "title": "Initializing a Factor Graph",
    "category": "section",
    "text": "using Caesar, RoME, Distributions\n\n# start with an empty factor graph object\nfg = initfg()"
},

{
    "location": "concepts/building_graphs/#Adding-to-the-Graph-1",
    "page": "Building Factor Graphs",
    "title": "Adding to the Graph",
    "category": "section",
    "text": "Factor graphs are made of two constituent parts:Variables\nFactors"
},

{
    "location": "concepts/building_graphs/#Variables-1",
    "page": "Building Factor Graphs",
    "title": "Variables",
    "category": "section",
    "text": "Variables (a.k.a. poses in localization terminology) are created in the same way  shown above for the landmark. Variables contain a label, a data type (e.g. in 2D RoME.Point2 or RoME.Pose2). Note that variables are solved - i.e. they are the product, what you wish to calculate when the solver runs - so you don\'t provide any measurements when creating them.# Add the first pose :x0\naddNode!(fg, :x0, Pose2)\n# Add a few more poses\nfor i in 1:10\n  addNode!(fg, Symbol(\"x$(i)\"), Pose2)\nend"
},

{
    "location": "concepts/building_graphs/#Factors-1",
    "page": "Building Factor Graphs",
    "title": "Factors",
    "category": "section",
    "text": "Factors are algebraic relationships between variables based on data cues such as sensor measurements. Examples of factors are absolute GPS readings (unary factors/priors) and odometry changes between pose variables. All factors encode a stochastic measurement (measurement + error), such as below, where a prior is defined against x0 with a normal distribution centered around [0,0,0]."
},

{
    "location": "concepts/building_graphs/#Priors-1",
    "page": "Building Factor Graphs",
    "title": "Priors",
    "category": "section",
    "text": "# Add at a fixed location Prior to pin :x0 to a starting location (0,0,pi/6.0)\naddFactor!(fg, [:x0], IIF.Prior( MvNormal([0; 0; pi/6.0], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))"
},

{
    "location": "concepts/building_graphs/#Factors-Between-Variables-1",
    "page": "Building Factor Graphs",
    "title": "Factors Between Variables",
    "category": "section",
    "text": "# Add odometry indicating a zigzag movement\nfor i in 1:10\n  pp = Pose2Pose2(MvNormal([10.0;0; (i % 2 == 0 ? -pi/3 : pi/3)], Matrix(Diagonal([0.1;0.1;0.1].^2))))\n  addFactor!(fg, [Symbol(\"x$(i-1)\"); Symbol(\"x$(i)\")], pp )\nend"
},

{
    "location": "concepts/building_graphs/#When-to-Create-New-Pose-Variables-1",
    "page": "Building Factor Graphs",
    "title": "When to Create New Pose Variables",
    "category": "section",
    "text": "Consider a robot traversing some area while exploring, localizing, and wanting to find strong loop-closure features for consistent mapping.  The creation of new poses and landmark variables is a trade-off in computational complexity and marginalization errors made during factor graph construction.  Common triggers for new poses are:Time-based trigger (eg. new pose a second or 5 minutes if stationary)\nDistance traveled (eg. new pose every 0.5 meters)\nRotation angle (eg. new pose every 15 degrees)Computation will progress faster if poses and landmarks are very sparse.  To extract the benefit of dense reconstructions, one approach is to use the factor graph as sparse index in history about the general progression of the trajectory and use additional processing from dense sensor data for high-fidelity map reconstructions.  Either interpolations, or better direct reconstructions from inertial data can be used for dense reconstruction.For completeness, one could also re-project the most meaningful measurements from sensor measurements between pose epochs as though measured from the pose epoch.  This approach essentially marginalizes the local dead reckoning drift errors into the local interpose re-projections, but helps keep the pose count low.In addition, see fixed-lag discussion for limiting during inference the number of fluid variables manually to a user desired count."
},

{
    "location": "concepts/building_graphs/#Variables-and-Factors-Available-in-Caesar-1",
    "page": "Building Factor Graphs",
    "title": "Variables and Factors Available in Caesar",
    "category": "section",
    "text": ""
},

{
    "location": "concepts/building_graphs/#Variables-Available-in-Caesar-1",
    "page": "Building Factor Graphs",
    "title": "Variables Available in Caesar",
    "category": "section",
    "text": "You can check for the latest variable types by running the following in your terminal:using RoME, Caesar\nsubtypes(IncrementalInference.InferenceVariable)Note: This has been made available as IncrementalInference.getCurrentWorkspaceVariables() in IncrementalInference v0.4.4.The current list of available variable types is:RoME.Point2 - A 2D coordinate consisting of [x, y, theta]\nRoME.Pose2 - A 2D coordinate and a rotation (i.e. bearing) consisting of [x, y, z, and theta]\nRoME.DynPoint2 - A 2D coordinate and linear velocities\nRoME.DynPose2 - A 2D coordinate, linear velocities, and a rotation\nRoME.Point3 - A 3D coordinate consisting of [x, y, z]\nRoME.Pose3 - A 3D coordinate and 3 associated rotations consisting of [x, y, z, theta, phi, psi]\nRoME.InertialPose3 - A 3D coordinate and rotation pose along with velocity and IMU bias calibration termsNote several more variable and factors types have been implemented which will over time be incorporated into standard RoME release.  Please open an issue with JuliaRobotics/RoME.jl for specific requests, problems, or suggestions.  Contributions are also welcome."
},

{
    "location": "concepts/building_graphs/#Factors-Available-in-Caesar-1",
    "page": "Building Factor Graphs",
    "title": "Factors Available in Caesar",
    "category": "section",
    "text": "You can check for the latest factor types by running the following in your terminal:using RoME, Caesar\nprintln(\"- Singletons (priors): \")\nprintln.(sort(string.(subtypes(IncrementalInference.FunctorSingleton))));\nprintln(\"- Pairwise (variable constraints): \")\nprintln.(sort(string.(subtypes(IncrementalInference.FunctorPairwise))));\nprintln(\"- Pairwise (variable minimization constraints): \")\nprintln.(sort(string.(subtypes(IncrementalInference.FunctorPairwiseMinimize))));Note: This has been made available as IncrementalInference.getCurrentWorkspaceFactors() in IncrementalInference v0.4.4.The current factor types that you will find in the examples are (there are many aside from these):RoME.Prior - A singleton indicating a prior on a variable\nRoME.Point2Point2 - A factor between two 2D points\nRoME.Point2Point2WorldBearing - A factor between two 2D points with bearing\nRoME.Pose2Point2Bearing - A factor between two 2D points with bearing\nRoME.Pose2Point2BearingRange - A factor between two 2D points with bearing and range\nRoME.Pose2Point2Range - A factor between a 2D pose and a 2D point, with range\nRoME.Pose2Pose2 - A factor between two 2D poses\nRoME.Pose3Pose3 - A factor between two 3D poses\nRoME.IntertialPose3 - A factor between two 3D IMU sensor poses"
},

{
    "location": "concepts/building_graphs/#Querying-the-FactorGraph-1",
    "page": "Building Factor Graphs",
    "title": "Querying the FactorGraph",
    "category": "section",
    "text": "There are a variety of functions to query the factor graph, please refer to Function Reference for details.A quick summary of the variables in the factor graph can be retrieved with:# List variables\nls(fg)\n# List factors attached to x0\nls(fg, :x0)\n# TODO: Provide an overview of getVal, getVert, getBW, getVertKDE, etc."
},

{
    "location": "concepts/building_graphs/#Solving-Graphs-1",
    "page": "Building Factor Graphs",
    "title": "Solving Graphs",
    "category": "section",
    "text": "When you have built the graph, you can call the solver to perform inference with the following:# Perform inference\nbatchSolve!(fg)"
},

{
    "location": "concepts/building_graphs/#Peeking-at-Results-1",
    "page": "Building Factor Graphs",
    "title": "Peeking at Results",
    "category": "section",
    "text": "Once you have solved the graph, you can review the full marginal with:X0 = getVertKDE(fg, :x0) # Get the raw KDE\n# Evaluate the marginal density function just for fun at [0.01, 0, 0].\nX0([0.01, 0, 0])For finding the MAP value in the density functions, you can use getKDEMax or getKDEMean. Here we are asking for the MAP values for all the variables in the factor graph:verts = ls(fg)\nmap(v -> println(\"$v : $(getKDEMax(getVertKDE(fg, v)))\"), verts[1]);Also see built-in function printgraphmax(fg) which performs a similar function."
},

{
    "location": "concepts/building_graphs/#Plotting-1",
    "page": "Building Factor Graphs",
    "title": "Plotting",
    "category": "section",
    "text": "Once the graph has been built, a simple plot of the values can be produced with RoMEPlotting.jl. For example:using RoMEPlotting\n\ndrawPoses(fg)\n# If you have landmarks, you can call drawPosesLandms(fg)\n\n# Draw the KDE for x0\nplotKDE(fg, :x0)\n# Draw the KDE\'s for x0 and x1\nplotKDE(fg, [:x0, :x1])"
},

{
    "location": "concepts/building_graphs/#Next-Steps-1",
    "page": "Building Factor Graphs",
    "title": "Next Steps",
    "category": "section",
    "text": "Although the above graph demonstrates the fundamental operations, it\'s not particularly useful. Take a look at Hexagonal Example for a complete example that builds on these operations."
},

{
    "location": "concepts/building_graphs/#Extending-Caesar-with-New-Variables-and-Factors-1",
    "page": "Building Factor Graphs",
    "title": "Extending Caesar with New Variables and Factors",
    "category": "section",
    "text": "A question that frequently arises is how to design custom variables and factors to solve a specific type of graph. One strength of Caesar is the ability to incorporate new variables and factors at will. Please refer to Adding Factors for more information on creating your own factors."
},

{
    "location": "concepts/arena_visualizations/#",
    "page": "Arena Visualization",
    "title": "Arena Visualization",
    "category": "page",
    "text": ""
},

{
    "location": "concepts/arena_visualizations/#Visualization-with-Arena.jl-1",
    "page": "Arena Visualization",
    "title": "Visualization with Arena.jl",
    "category": "section",
    "text": "Caesar.jl uses the Arena.jl package for all the visualization requirements.  This part of the documentation discusses the robotic visualization aspects supported by Arena.jl. Arena.jl supports a wide variety of general visualization as well as developer visualization tools more focused on research and development.Over time, Caesar.jl has used a least three different 3D visualization technologies, with the most recent based on WebGL and three.js by means of the MeshCat.jl package. The previous incarnation used a client side installation of VTK  by means of the DrakeVisualizer.jl and Director libraries. Different 2D plotting libraries have also been used, with evolutions to improve usability for a wider user base. Each epoch has aimed at reducing dependencies and increasing multi-platform support.The sections below discuss 2D and 3D visualization techniques available to the Caesar.jl robot navigation system. Visualization examples will be seen throughout the Caesar.jl package documentation.Note that all visualizations used to be part of the Caesar.jl package itself, but was separated out to Arena.jl in early 2018."
},

{
    "location": "concepts/arena_visualizations/#Installation-1",
    "page": "Arena Visualization",
    "title": "Installation",
    "category": "section",
    "text": "The current version of Arena has a rather large VTK dependency (which compile just fine on Ubuntu/Debian, or maybe even MacOS) wrapped in the DrakeVisualizer.jl package.  This requires the following preinstalled packages:    sudo apt-get install libvtk5-qt4-dev python-vtkNOTE Smaller individual 2D packages can be installed instead – i.e.:Pkg.add(\"RoMEPlotting\")For the full 2D/3D visualization tools used by Caesar.jl–-in a Julia or (JuliaPro) terminal/REPL–-type:julia> Pkg.add(\"Arena\")NOTE Current development will allow the user to choose a three.js WebGL based viewer instead MeshCat.jl."
},

{
    "location": "concepts/arena_visualizations/#D-Visualization-1",
    "page": "Arena Visualization",
    "title": "2D Visualization",
    "category": "section",
    "text": "2D plot visualizations, provided by RoMEPlotting.jl and KernelDensityEstimatePlotting.jl, are generally useful for repeated analysis of a algorithm or data set being studied. These visualizations are often manipulated to emphasize particular aspects of mobile platform navigation. Arena.jl is intended to simplify the process 2D plotting for robot trajectories in two or three dimensions. The visualizations are also intended to help with subgraph plotting for finding loop closures in data or compare two datasets."
},

{
    "location": "concepts/arena_visualizations/#Hexagonal-2D-SLAM-example-visualization-1",
    "page": "Arena Visualization",
    "title": "Hexagonal 2D SLAM example visualization",
    "category": "section",
    "text": "The major 2D plotting functions between RoMEPlotting.jl:drawPoses\ndrawPosesLandms\ndrawSubmapsand KernelDensityEstimatePlotting.jl:plotKDE / plot(::KernelDensityEstimate)This simplest example for visualizing a 2D robot trajectory–-such as first running the Hexagonal 2D SLAM example–-# Assuming some fg::FactorGraph has been loaded/constructed\n# ...\n\nusing RoMEPlotting\n\n# For Juno/Jupyter style use\npl = drawPosesLandms(fg)\n\n# For scripting use-cases you can export the image\nGadfly.draw(PDF(\"/tmp/test.pdf\", 20cm, 10cm),pl)  # or PNG(...)(Image: test)"
},

{
    "location": "concepts/arena_visualizations/#Density-Contour-Map-1",
    "page": "Arena Visualization",
    "title": "Density Contour Map",
    "category": "section",
    "text": "KernelDensityEstimatePlotting (as used in RoMEPlotting) provides an interface to visualize belief densities as counter plots. The following basic example shows some of features of the API, where plotKDE(..., dims=[1;2]) implies the marginal over variables (x,y):using RoME, Distributions\nusing RoMEPlotting\n\nfg = initfg()\naddNode!(fg, :x0, Pose2)\naddFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), eye(3))))\naddNode!(fg, :x1, Pose2)\naddFactor!(fg, [:x0;:x1], Pose2Pose2(MvNormal([10.0;0;0], eye(3))))\n\nensureAllInitialized!(fg)\n\n# plot one contour density\nplX0 = plotKDE(fg, :x0, dims=[1;2])\n# using Gadfly; Gadfly.draw(PNG(\"/tmp/testX0.png\",20cm,10cm),plX0)(Image: test)The contour density relates to the distribution of marginal samples as seen with this Gadfly.jl package histogram comparison.pl1 = drawPoses(fg, to=0);\nX0 = getVal(fg, :x0);\npl2 = Gadfly.plot(x=X0[1,:],y=X0[2,:], Geom.hexbin);\nplH = hstack(pl1, pl2)\n# Gadfly.draw(PNG(\"/tmp/testH.png\",20cm,10cm),plH)(Image: testh)NOTE Red and Green lines represent Port and Starboard direction of Pose2, respectively.Multiple beliefs can be plotted at the same time, while setting levels=4 rather than the default value:plX1 = plotKDE(fg, [:x0; :x1], dims=[1;2], levels=4)\n# Gadfly.draw(PNG(\"/tmp/testX1.png\",20cm,10cm),plX1)(Image: testx1)One dimensional (such as Θ) or a stack of all plane projections is also available:plTh = plotKDE(fg, [:x0; :x1], dims=[3], levels=4)\n# Gadfly.draw(PNG(\"/tmp/testTh.png\",20cm,10cm),plTh)(Image: testth)plAll = plotKDE(fg, [:x0; :x1], levels=3)\n# Gadfly.draw(PNG(\"/tmp/testX1.png\",20cm,15cm),plAll)(Image: testall)NOTE The functions hstack and vstack is provided through the Gadfly package and allows the user to build a near arbitrary composition of plots.Please see KernelDensityEstimatePlotting package source for more features."
},

{
    "location": "concepts/arena_visualizations/#D-Visualization-2",
    "page": "Arena Visualization",
    "title": "3D Visualization",
    "category": "section",
    "text": "Factor graphs of two or three dimensions can be visualized with the 3D visualizations provided by Arena.jl and it\'s dependencies. The 2D example above and also be visualized in a 3D space with the commands:vc = startdefaultvisualization() # to load a DrakeVisualizer/Director process instance\nvisualize(fg, vc, drawlandms=false)\n# visualizeallposes!(vc, fg, drawlandms=false)Here is a basic example of using visualization and multi-core factor graph solving:addprocs(2)\nusing Caesar, RoME, TransformUtils, Distributions\n\n# load scene and ROV model (might experience UDP packet loss LCM buffer not set)\nsc1 = loadmodel(:scene01); sc1(vc)\nrovt = loadmodel(:rov); rovt(vc)\n\ninitCov = 0.001*eye(6); [initCov[i,i] = 0.00001 for i in 4:6];\nodoCov = 0.0001*eye(6); [odoCov[i,i] = 0.00001 for i in 4:6];\nrangecov, bearingcov = 3e-4, 2e-3\n\n# start and add to a factor graph\nfg = identitypose6fg(initCov=initCov)\ntf = SE3([0.0;0.7;0.0], Euler(pi/4,0.0,0.0) )\naddOdoFG!(fg, Pose3Pose3(MvNormal(veeEuler(tf), odoCov) ) )\n\naddLinearArrayConstraint(fg, (4.0, 0.0), :x0, :l1, rangecov=rangecov,bearingcov=bearingcov)\naddLinearArrayConstraint(fg, (4.0, 0.0), :x1, :l1, rangecov=rangecov,bearingcov=bearingcov)\n\nsolveBatch!(fg)\n\nusing Arena\n\nvc = startdefaultvisualization()\nvisualize(fg, vc, drawlandms=true, densitymeshes=[:l1;:x2])\nvisualizeDensityMesh!(vc, fg, :l1)\n# visualizeallposes!(vc, fg, drawlandms=false)"
},

{
    "location": "concepts/zmq/#",
    "page": "Using Caesar\'s Multi-Language Support",
    "title": "Using Caesar\'s Multi-Language Support",
    "category": "page",
    "text": ""
},

{
    "location": "concepts/zmq/#Caesar-Multi-Language-Support-1",
    "page": "Using Caesar\'s Multi-Language Support",
    "title": "Caesar Multi-Language Support",
    "category": "section",
    "text": "Caesar allows other languages to construct and solve graphs via it\'s ZMQ interface."
},

{
    "location": "concepts/zmq/#Concepts-1",
    "page": "Using Caesar\'s Multi-Language Support",
    "title": "Concepts",
    "category": "section",
    "text": "TODO: ZMQ discussion."
},

{
    "location": "concepts/zmq/#Applications-1",
    "page": "Using Caesar\'s Multi-Language Support",
    "title": "Applications",
    "category": "section",
    "text": "The flagship example of an implementation of this is the Caear C/C++ interface that can be found here GraffCPP."
},

{
    "location": "concepts/adding_variables_factors/#",
    "page": "Adding New Variables and Factors",
    "title": "Adding New Variables and Factors",
    "category": "page",
    "text": ""
},

{
    "location": "concepts/adding_variables_factors/#Creating-New-Variables-and-Factors-1",
    "page": "Adding New Variables and Factors",
    "title": "Creating New Variables and Factors",
    "category": "section",
    "text": "In most scenarios, the existing variables and factors should be sufficient for most robotics applications. Caesar however, is extensible and allows you to easily incorporate your own variable and factor types for specialized applications."
},

{
    "location": "concepts/adding_variables_factors/#Considerations-1",
    "page": "Adding New Variables and Factors",
    "title": "Considerations",
    "category": "section",
    "text": "A couple of important points:You do not need to modify/fork/edit internal Caesar/RoME/IncrementalInference source code to introduce new variable and factor types!\nAs long as the factors exist in the working space when the solver is run, the factors are automatically used – this is possible due to Julia\'s multiple dispatch design\nCaesar is designed to allow you to add new variables and factors to your own independent repository and incorporate them at will at compile-time or even run-time\nResidual function definitions for new factors types use a callable struct (a.k.a functor) architecture to simultaneously allow:  \nMultiple dispatch (i.e. \'polymorphic\' behavior)\nMeta-data and in-place memory storage for advanced and performant code\nAn outside callback implementation style\nIn most robotics scenarios, there is no need for new variables or factors:\nVariables have various mechanisms that allow you to attach data to them, e.g. raw sensory data or identified April tags, so you do not need to create a new variable type just to store data\nNew variables are required only if you are representing a new state - TODO: Example of needed state\nNew factors are needed if:\nYou need to represent a constraint for a variable (known as a singleton) and that constraint type doesn\'t exist\nYou need to represent a constraint between two variables and that constraint type doesn\'t exist"
},

{
    "location": "concepts/adding_variables_factors/#Getting-Started-1",
    "page": "Adding New Variables and Factors",
    "title": "Getting Started",
    "category": "section",
    "text": "We suggest the following design pattern for developing and building new factors:You have reviewed the variable and factor types available in Caesar, RoME, and IncrementalInference and a new type is required - please see Building and Solving Graphs if you want to review what is currently available\nCreate a GitHub repository to store the new types\nCreate your new variable types\nCreate your new factor types\nImplement unit tests to validate the correct operation of the types\nSet up your solver to make use the custom types1.1. This is much easier than it soundsIf the code is public and may be useful to the community, we ask if you could submit an issue against Caesar with information about the new types and the repository. Ideally we\'d like to continually improve the core code and fold in community contributions.The remainder of this section discusses each of these steps."
},

{
    "location": "concepts/adding_variables_factors/#Reviewing-the-Existing-Types-1",
    "page": "Adding New Variables and Factors",
    "title": "Reviewing the Existing Types",
    "category": "section",
    "text": "Please see Building and Solving Graphs to review what variables and factors are currently supported."
},

{
    "location": "concepts/adding_variables_factors/#Creating-a-Repository-1",
    "page": "Adding New Variables and Factors",
    "title": "Creating a Repository",
    "category": "section",
    "text": "You can fork the following template repository to construct your own Caesar Variable and Factor Examples.If this repository is going to be used for development of the new variables/factors as well as for the experiment (i.e. the code that builds the graph and solves it), you should probably start a simple end-to-end test that validates a basic version of your experimental setup (e.g. ):#### This example is a basic test of the new variables and factors\n#### that are added in this repo. The example is derived from\n#### the hexagonal test example.\n\nusing Caesar, RoME\nusing Caesar_VariableFactorExamples # Your new variable/factor repository\n# Using plotting for experiment validation\nusing RoMEPlotting\n\n# 1. Init factor graph\n#TODO\n\n# 2. Add variables\n#TODO\n\n# 3. Add factors\n# 3a. Add a new test prior\n#TODO\n# 3b. Add new types of odometry factors.\n#TODO\n\n# 4. Solve graph\nbatchSolve!(fg)\n\n# 5. Graph solution - assuming that you have this open in Atom.\ndrawPoses(fg)"
},

{
    "location": "concepts/adding_variables_factors/#Creating-New-Variables-1",
    "page": "Adding New Variables and Factors",
    "title": "Creating New Variables",
    "category": "section",
    "text": "All variables have to derive from IncrementalInference.InferenceVariable.What you need to build in the variable:dims - This is used during computation and defines the degrees of freedom (dimensions) for variable\nlabels - This a required field, although it does not need to be populated. It consists of unique, indexable string identifiers, such as \'POSE\', \'LANDMARK\'. It assists with querying the data efficiently in large systems when using the database layer.  You can then also add any additional fields that you would like to use for saving state information in variable. Note that these fields must be serializable as both JSON and Protobufs. Although you don\'t need to validate this, please keep the fields fairly simple and avoid complex structures with optional fields. TBD - provide a compatibility check for serialization and a docpage on it.In a trivial example of Pose2:Our dimensions would then be 3: X, Y, theta\nThe labels for Pose2 could be \"POSE\""
},

{
    "location": "concepts/adding_variables_factors/#Creating-New-Factors-1",
    "page": "Adding New Variables and Factors",
    "title": "Creating New Factors",
    "category": "section",
    "text": "All factors inherit from one of the following types, depending on their function:FunctorSingleton: FunctorSingletons are priors (unary factors) that provide an absolute constraint for a single variable. A simple example of this is an absolute GPS prior, or equivalently a (0, 0, 0) starting location in a RoME.Pose2 scenario.\nRequires: A getSample function\nFunctorPairwiseMinimize: FunctorPairwiseMinimize are relative factors that introduce an algebraic relationship between two or more variables. A simple example of this is an odometry factor between two pose variables, or a range factor indicating the range between a pose and another variable.\nRequires: A getSample function and a residual function definition\nThe minimize suffix specifies that the residual function of this factor will be enforced by numerical minimization (find me the minimum of this function)\nFunctorPairwise: FunctorPairwise are relative factors that introduce algebraic relationships between two or more variables. They are the same as FunctorPairwiseMinimize, however they use root finding to find the zero crossings (rather than numerical minimization).\nRequires: A getSample function and a residual function definitionHow do you decide which to use?If you are creating factors for world-frame information that will be tied to a single variable, inherit from FunctorSingleton\nGPS coordinates should be priors\nIf you are creating factors for local-frame relationships between variables, inherit from FunctorPairwiseMinimize\nOdometry and bearing deltas should be introduced as pairwise factors and should be local frameTBD: sUsers should start with FunctorPairwiseMinimize, discuss why and when they should promote their factors to FunctorPairwise.Note: FunctorPairwiseMinimize does not imply that the overall inference algorithm only minimizes an objective function. The Multi-model iSAM algorithm is built around fixed-point analysis. Minimization is used here to locally enforce the residual function.What you need to build in the new factor:A struct for the factor itself\nA sampler function to return measurements from the random ditributions\nIf you are building a FunctorPairwiseMinimize or a FunctorPairwise you need to define a residual function to introduce the relative algebraic relationship between the variables\nMinimization function should be lower-bounded and smooth\nA packed type of the factor which must be named Packed[Factor name], and allows the factor to be packed/transmitted/unpacked\nSerialization and deserialization methods\nThese are convert functions that pack and unpack the factor (which may be highly complex) into serialization-compatible formats\nAs the factors are mostly comprised of distributions (of type SamplableBelief), functions are provided to pack and unpack the distributions:\nPacking: To convert from a SamplableBelief to a string, use string(::SamplableBelief)::String\nUnpacking: To convert from string back to a SamplableBelief, use extractdistribution(::String)::SamplableBelief  An example of this is the Pose2Point2BearingRange, which provides a bearing+range relationship between a 2D pose and a 2D point."
},

{
    "location": "concepts/adding_variables_factors/#Pose2Point2BearingRange-Struct-1",
    "page": "Adding New Variables and Factors",
    "title": "Pose2Point2BearingRange Struct",
    "category": "section",
    "text": "mutable struct Pose2Point2BearingRange{B <: IIF.SamplableBelief, R <: IIF.SamplableBelief} <: IncrementalInference.FunctorPairwise\n    bearing::B\n    range::R\n    Pose2Point2BearingRange{B,R}() where {B,R} = new{B,R}()\n    Pose2Point2BearingRange{B,R}(x1::B,x2::R) where {B <: IIF.SamplableBelief,R <: IIF.SamplableBelief} = new{B,R}(x1,x2)\nend\n# Convenient constructor\nPose2Point2BearingRange(x1::B,x2::R) where {B <: IIF.SamplableBelief,R <: IIF.SamplableBelief} = Pose2Point2BearingRange{B,R}(x1,x2)"
},

{
    "location": "concepts/adding_variables_factors/#Pose2Point2BearingRange-Sampler-1",
    "page": "Adding New Variables and Factors",
    "title": "Pose2Point2BearingRange Sampler",
    "category": "section",
    "text": "# Return N samples from the two distributions\nfunction getSample(pp2br::Pose2Point2BearingRange, N::Int=1)\n  smpls = zeros(2, N)\n  smpls[1,:] = rand(pp2br.bearing, N)[:]\n  smpls[2,:] = rand(pp2br.range, N)[:]\n  return (smpls,)\nend"
},

{
    "location": "concepts/adding_variables_factors/#Pose2Point2BearingRange-Residual-Function-(Functor)-1",
    "page": "Adding New Variables and Factors",
    "title": "Pose2Point2BearingRange Residual Function (Functor)",
    "category": "section",
    "text": "# define the conditional probability constraint\nfunction (pp2br::Pose2Point2BearingRange)(res::Array{Float64},\n        userdata::FactorMetadata,\n        idx::Int,\n        meas::Tuple{Array{Float64,2}},\n        xi::Array{Float64,2},\n        lm::Array{Float64,2} )\n  #\n  res[1] = lm[1,idx] - (meas[1][2,idx]*cos(meas[1][1,idx]+xi[3,idx]) + xi[1,idx])\n  res[2] = lm[2,idx] - (meas[1][2,idx]*sin(meas[1][1,idx]+xi[3,idx]) + xi[2,idx])\n  nothing\nend"
},

{
    "location": "concepts/adding_variables_factors/#Pose2Point2BearingRange-Packing-and-Unpacking-1",
    "page": "Adding New Variables and Factors",
    "title": "Pose2Point2BearingRange Packing and Unpacking",
    "category": "section",
    "text": "The packing structure:mutable struct PackedPose2Point2BearingRange <: IncrementalInference.PackedInferenceType\n    bearstr::String\n    rangstr::String\n    PackedPose2Point2BearingRange() = new()\n    PackedPose2Point2BearingRange(s1::AS, s2::AS) where {AS <: AbstractString} = new(string(s1),string(s2))\nendThe packing and unpacking converters (note the use of string and extractdistribution):function convert(::Type{PackedPose2Point2BearingRange}, d::Pose2Point2BearingRange{B, R}) where {B <: IIF.SamplableBelief, R <: IIF.SamplableBelief}\n  return PackedPose2Point2BearingRange(string(d.bearing), string(d.range))\nend\n\nfunction convert(::Type{Pose2Point2BearingRange}, d::PackedPose2Point2BearingRange)\n # where {B <: IIF.SamplableBelief, R <: IIF.SamplableBelief}\n  Pose2Point2BearingRange(extractdistribution(d.bearstr), extractdistribution(d.rangstr))\nend"
},

{
    "location": "concepts/adding_variables_factors/#Unit-Tests-1",
    "page": "Adding New Variables and Factors",
    "title": "Unit Tests",
    "category": "section",
    "text": "What you should test:Creation of the factor\nSampling of the factor\nResidual testing\nSolving using the variables and factors\nSerialization and deserializationAn example of these tests can be seen for the trivial case shown in the example repo ExamplePrior Unit Tests."
},

{
    "location": "concepts/adding_variables_factors/#Using-your-Types-with-the-Caesar-Solver-1",
    "page": "Adding New Variables and Factors",
    "title": "Using your Types with the Caesar Solver",
    "category": "section",
    "text": "As above, as long as you bring your factors into the workspace, you should be able to use them in your experimental setup.You can validate this with the existence check code in Building and Solving Graphs.Note: This has been made available as IncrementalInference.getCurrentWorkspaceVariables() and IncrementalInference.getCurrentWorkspaceFactors()in IncrementalInference v0.4.4."
},

{
    "location": "concepts/adding_variables_factors/#Contributing-to-Community-1",
    "page": "Adding New Variables and Factors",
    "title": "Contributing to Community",
    "category": "section",
    "text": "We really appreciate any contributions, so if you have developed variables and factors that may be useful to the community, please write up an issue in Caesar.jl with a link to your repo and a short description of the use-case(s)."
},

{
    "location": "concepts/database_interactions/#",
    "page": "Using Caesar Database Operation",
    "title": "Using Caesar Database Operation",
    "category": "page",
    "text": ""
},

{
    "location": "concepts/database_interactions/#Common-Data-Persistence-and-Inference-1",
    "page": "Using Caesar Database Operation",
    "title": "Common Data Persistence and Inference",
    "category": "section",
    "text": "Coming soon!"
},

{
    "location": "examples/examples/#",
    "page": "Caesar Examples",
    "title": "Caesar Examples",
    "category": "page",
    "text": ""
},

{
    "location": "examples/examples/#Examples-1",
    "page": "Caesar Examples",
    "title": "Examples",
    "category": "section",
    "text": ""
},

{
    "location": "examples/examples/#Basics-1",
    "page": "Caesar Examples",
    "title": "Basics",
    "category": "section",
    "text": "The following examples demonstrate the conceptual operation of Caesar, highlighting specific features of the framework and its use."
},

{
    "location": "examples/examples/#Continuous-Scalar-1",
    "page": "Caesar Examples",
    "title": "Continuous Scalar",
    "category": "section",
    "text": "This abstract example illustrates how IncrementalInference enables algebraic relations between stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information.Continuous Scalar Example"
},

{
    "location": "examples/examples/#Hexagonal-2D-1",
    "page": "Caesar Examples",
    "title": "Hexagonal 2D",
    "category": "section",
    "text": "A simple 2D robot trajectory example is expanded below using techniques developed in simultaneous localization and mapping (SLAM).Hexagonal 2D Example"
},

{
    "location": "examples/examples/#Fixed-Lag-Solving-Hexagonal2D-Revisited-1",
    "page": "Caesar Examples",
    "title": "Fixed-Lag Solving - Hexagonal2D Revisited",
    "category": "section",
    "text": "Hexagonal Fixed-Lag"
},

{
    "location": "examples/examples/#A-Under-Constrained-Solution-(unforced-multimodality)-1",
    "page": "Caesar Examples",
    "title": "A Under-Constrained Solution (unforced multimodality)",
    "category": "section",
    "text": "This tutorial describes a range-only system where there are always more variable dimensions than range measurements made. The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point – other examples show inference results where highly non-Gaussian error distributions are used.Multi-modal range only example (click here or image for full Vimeo):   <a href=\"http://vimeo.com/190052649\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"640\" border=\"0\" /></a>Multi-Modal Under-Constrained Example"
},

{
    "location": "examples/examples/#Uncertain-Data-Associations,-a-Multi-Modal-Solution-(forced-multi-hypothesis)-1",
    "page": "Caesar Examples",
    "title": "Uncertain Data Associations, a Multi-Modal Solution (forced multi-hypothesis)",
    "category": "section",
    "text": "Documentation in progress, in the mean time please see the addFactor!(..., multihypo=[1.0; 0.5;0.5]) feature for 50/50 uncertainty. Similarly for trinary or higher multi-hypotheses per factor.TODO: add example."
},

{
    "location": "examples/examples/#Adding-Factors-Simple-Factor-Design-1",
    "page": "Caesar Examples",
    "title": "Adding Factors - Simple Factor Design",
    "category": "section",
    "text": "Caesar can be extended with new variables and factors without changing the core code. An example of this design pattern is provided in this example.Defining New Variables and Factor"
},

{
    "location": "examples/examples/#Adding-Factors-DynPose-Factor-1",
    "page": "Caesar Examples",
    "title": "Adding Factors - DynPose Factor",
    "category": "section",
    "text": "Intermediate Example: Adding Dynamic Factors and Variables"
},

{
    "location": "examples/examples/#Application-Examples-and-Demos-1",
    "page": "Caesar Examples",
    "title": "Application Examples and Demos",
    "category": "section",
    "text": ""
},

{
    "location": "examples/examples/#Multi-session-Use-case-1",
    "page": "Caesar Examples",
    "title": "Multi-session Use-case",
    "category": "section",
    "text": "Multi-session Turtlebot example of the second floor in the Stata Center:   <img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/turtlemultisession.gif\" alt=\"Turtlebot Multi-session animation\" width=\"480\" border=\"0\" /></a>"
},

{
    "location": "examples/examples/#Simulated-Ambiguous-SONAR-in-3D-1",
    "page": "Caesar Examples",
    "title": "Simulated Ambiguous SONAR in 3D",
    "category": "section",
    "text": "Intersection of ambiguous elevation angle from planar SONAR sensor:   <a href=\"http://vimeo.com/198237738\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovasfm02.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>Bi-modal belief   <a href=\"http://vimeo.com/198872855\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/imgs/rovyaw90.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"480\" border=\"0\" /></a>"
},

{
    "location": "examples/examples/#More-Examples-1",
    "page": "Caesar Examples",
    "title": "More Examples",
    "category": "section",
    "text": "Please see examples folders for Caesar and RoME for more examples, with expanded documentation in the works."
},

{
    "location": "examples/basic_continuousscalar/#",
    "page": "ContinuousScalar",
    "title": "ContinuousScalar",
    "category": "page",
    "text": ""
},

{
    "location": "examples/basic_continuousscalar/#Tutorials-1",
    "page": "ContinuousScalar",
    "title": "Tutorials",
    "category": "section",
    "text": ""
},

{
    "location": "examples/basic_continuousscalar/#IncrementalInference.jl-ContinuousScalar-1",
    "page": "ContinuousScalar",
    "title": "IncrementalInference.jl ContinuousScalar",
    "category": "section",
    "text": "This tutorial illustrates how IncrementalInference enables algebraic relations (residual functions) between multiple stochastic variables, and how a final posterior belief estimate is calculated from several pieces of information. The application of this tutorial is presented in abstract from which the user is free to imagine any system of relationships:  For example, a robot driving in a one dimensional world; or a time traveler making uncertain jumps forwards and backwards in time. The tutorial implicitly shows a multi-modal uncertainty introduced and transmitted. The tutorial also illustrates consensus through an additional piece of information, which reduces all stochastic variable marginal beliefs to unimodal only beliefs. The example will also illustrate the use of non-Gaussian beliefs and global inference. Lastly, the tutorial demonstrates how automatic initialization of variables works.This tutorial requires IncrementalInference v0.3.0+, RoME v0.1.0, RoMEPlotting packages be installed. In addition, the optional GraphViz package will allow easy visualization of the FactorGraph object structure.To start, the two major mathematical packages are brought into scope.using IncrementalInference\n# using Distributions # automatically reexported by IncrementalInferenceGuidelines for developing your own functions are discussed here in Adding Variables and Factors, and we note that mechanizations and manifolds required for robotic simultaneous localization and mapping (SLAM) has been tightly integrated with the expansion package RoME.jl.The next step is to describe the inference problem with a graphical model of type IncrementalInference.FactorGraph. The first step is to create an empty factor graph object and start populating it with variable nodes. The variable nodes are identified by Symbols, namely :x0, :x1, :x2, :x3.# Start with an empty factor graph\nfg = emptyFactorGraph()\n\n# add the first node\naddNode!(fg, :x0, ContinuousScalar)\n\n# this is unary (prior) factor and does not immediately trigger autoinit of :x0.\naddFactor!(fg, [:x0], Prior(Normal(0,1)))Factor graphs are bipartite graphs with factors that act as mathematical structure between interacting variables. After adding node :x0, a singleton factor of type Prior (which was defined by the user earlier) is \'connected to\' variable node :x0. This unary factor is taken as a Distributions.Normal distribution with zero mean and a standard devitation of 1. GraphViz.jl can be used to visualize the factor graph structure, although the package is not installed by default. Furthermore, the writeGraphPdf member definition is given at the end of this tutorial, which allows the user to store the graph image in graphviz supported image types.Graphs.plot(fg.g)\n# writeGraphPdf(fg, file=\"fgx01.pdf\") # file=\"fgx01.png\"The two node factor graph is shown in the image below.<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/fgx0.png\" width=\"120\" border=\"0\" />\n</p>Automatic initialization of variables depend on how the factor graph model is constructed. This tutorial demonstrates this behavior by first showing that :x0 is not initialized:@show isInitialized(fg, :x0) # falseWhy is :x0 not initialized? Since no other variable nodes have been \'connected to\' (or depend) on :x0 and future intentions of the user are unknown, the initialization of :x0 is deferred until the latest possible moment. IncrementalInference.jl assumes that the user will generally populate new variable nodes with most of the associated factors before moving to the next variable. By delaying initialization of a new variable (say :x0) until a second newer uninitialized variable (say :x1) depends on :x0, the IncrementalInference algorithms hope to then initialize :x0 with the more information from previous and surrounding variables and factors. Also note that initialization of variables is a local operation based only on the neighboring nodes – global inference will over the entire graph is shows later in this tutorial.By adding :x1 and connecting it through the LinearOffset and Normal distributed factor, the automatic initialization of :x0 is triggered.addNode!(fg, :x1, ContinuousScalar)\n# P(Z | :x1 - :x0 ) where Z ~ Normal(10,1)\naddFactor!(fg, [:x0, :x1], LinearOffset(Normal(10.0,1)))\n@show isInitialized(fg, :x0) # trueNote that the automatic initialization of :x0 is aware that :x1 is not initialized and therefore only used the Prior(Normal(0,1)) unary factor to initialize the marginal belief estimate for :x0. The structure of the graph has now been updated to two variable nodes and two factors.<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/fgx01.png\" width=\"240\" border=\"0\" />\n</p>Global inference requires that the entire factor graph be initialized before the numerical belief computation algorithms can be performed. Notice how the new :x1 variable is not yet initialized:@show isInitialized(fg, :x1) # falseThe RoMEPlotting.jl package allows visualization (plotting) of the belief state over any of the variable nodes. Remember the first time executions are slow given required code compilation, and that future versions of these package will use more precompilation to reduce first execution running cost.using RoMEPlotting\n\nplotKDE(fg, :x0)<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/plx0.png\" width=\"360\" border=\"0\" />\n</p>By forcing the initialization of :x1 and plotting its belief estimate,ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1])the predicted influence of the P(Z| X1 - X0) = LinearOffset(Normal(10, 1)) is shown by the red trace.<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/plx01.png\" width=\"360\" border=\"0\" />\n</p>The red trace (predicted belief of :x1) is noting more than the approximated convolution of the current marginal belief of :x0 with the conditional belief described by P(Z | X1 - X0).Another ContinuousScalar variable :x2 is \'connected\' to :x1 through a more complicated MultiModalOffset likelihood function.addNode!(fg, :x2, ContinuousScalar)\nmmo = MultiModalOffset([Rayleigh(3); Uniform(30,55)], Categorical([0.4; 0.6]))\naddFactor!(fg, [:x1, :x2], mmo)<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/fgx012.png\" width=\"360\" border=\"0\" />\n</p>The mmo variable illustrates how a near arbitrary mixture probability distribution can be used as a conditional relationship between variable nodes in the factor graph. In this case, a 40%/60% balance of a Rayleigh and truncated Uniform distribution which acts as a multi-modal conditional belief. Interpret carefully what a conditional belief of this nature actually means.Following the tutorial\'s practical example frameworks (robot navigation or time travel), this multi-modal belief implies that moving from one of the probable locations in :x1 to a location in :x2 by some processes defined by mmo=P(Z | X2, X1) is uncertain to the same 40%/60% ratio. In practical terms, collapsing (through observation of an event) the probabilistic likelihoods of the transition from :x1 to :x2 may result in the :x2 location being at either 15-20, or 40-65-ish units. The predicted belief over :x2 is illustrated by plotting the predicted belief (green trace), after forcing initialization.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2])<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/plx012.png\" width=\"360\" border=\"0\" />\n</p>Adding one more variable :x3 through another LinearOffset(Normal(-50,1))addNode!(fg, :x3, ContinuousScalar)\naddFactor!(fg, [:x2, :x3], LinearOffset(Normal(-50, 1)))expands the factor graph to to four variables and four factors.<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/fgx0123.png\" width=\"480\" border=\"0\" />\n</p>This part of the tutorial shows how a unimodal likelihood (conditional belief) can transmit the bimodal belief currently contained in :x2.ensureAllInitialized!(fg)\nplotKDE(fg, [:x0, :x1, :x2, :x3])Notice the blue trace (:x3) is a shifted and slightly spread out version of the initialized belief on :x2, through the convolution with the conditional belief P(Z | X2, X3).<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/plx0123.png\" width=\"480\" border=\"0\" />\n</p>Global inference over the entire factor graph has still not occurred, and will at this stage produce roughly similar results to the predicted beliefs shown above. Only by introducing more information into the factor graph can inference extract more precise marginal belief estimates for each of the variables. A final piece of information added to this graph is a factor directly relating :x3 with :x0.addFactor!(fg, [:x3, :x0], LinearOffset(Normal(40, 1)))Pay close attention to what this last factor means in terms of the probability density traces shown in the previous figure. The blue trace for :x3 has two major modes, one that overlaps with :x0, :x1 near 0 and a second mode further to the left at -40. The last factor introduces a shift LinearOffset(Normal(40,1)) which essentially aligns the left most mode of :x3 back onto :x0.<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/fgx0123c.png\" width=\"480\" border=\"0\" />\n</p>This last factor forces a mode selection through consensus. By doing global inference, the new information obtained in :x3 will be equally propagated to :x2 where only one of the two modes will remain.Global inference is achieved with local computation using two function calls, as follows.tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# and visualization\nplotKDE(fg, [:x0, :x1, :x2, :x3])The resulting posterior marginal beliefs over all the system variables are:<p align=\"center\">\n<img src=\"https://raw.githubusercontent.com/JuliaRobotics/Caesar.jl/master/docs/src/assets/tutorials/ContinuousScalar/plx0123infr.png\" width=\"480\" border=\"0\" />\n</p>It is import to note that although this tutorial ends with all marginal beliefs having near Gaussian shape and are unimodal, that the package supports multi-modal belief estimates during both the prediction and global inference processes. In fact, many of the same underlying inference functions are involved with the automatic initialization process and the global multi-modal iSAM inference procedure. This concludes the ContinuousScalar tutorial particular to the IncrementalInference package."
},

{
    "location": "examples/basic_slamedonut/#",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Singular Ranges-only SLAM (Underdetermined System)",
    "category": "page",
    "text": ""
},

{
    "location": "examples/basic_slamedonut/#Singular-Ranges-only-SLAM-Solution-(i.e.-\"Under-Constrained\")-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Singular Ranges-only SLAM Solution (i.e. \"Under-Constrained\")",
    "category": "section",
    "text": "This tutorial describes a range-only system where there are always more variable dimensions than range measurements made. The error distribution over ranges could be nearly anything, but are restricted to Gaussian-only in this example to illustrate an alternative point – other examples show inference results where highly non-Gaussian error distributions are used. The one pre-baked result of this of this singular range-only illustration can be seen in this video:Multi-modal range only example (click here or image for full Vimeo):   <a href=\"http://vimeo.com/190052649\" target=\"_blank\"><img src=\"https://raw.githubusercontent.com/JuliaRobotics/IncrementalInference.jl/master/doc/images/mmisamvid01.gif\" alt=\"IMAGE ALT TEXT HERE\" width=\"640\" border=\"0\" /></a>This example is also available as a script here in RoME.jl."
},

{
    "location": "examples/basic_slamedonut/#REQUIRES-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "REQUIRES",
    "category": "section",
    "text": "RoME v0.2.5\nRoMEPlotting v0.0.2"
},

{
    "location": "examples/basic_slamedonut/#Loading-The-Data-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Loading The Data",
    "category": "section",
    "text": "Starting a Juno IDE or Julia REPL session, the ground truth positions for vehicle positions GTp and landmark positions GTl can be loaded into memory directly with these values:GTp = Dict{Symbol, Vector{Float64}}()\nGTp[:l100] = [0.0;0]\nGTp[:l101] = [50.0;0]\nGTp[:l102] = [100.0;0]\nGTp[:l103] = [100.0;50.0]\nGTp[:l104] = [100.0;100.0]\nGTp[:l105] = [50.0;100.0]\nGTp[:l106] = [0.0;100.0]\nGTp[:l107] = [0.0;50.0]\nGTp[:l108] = [0.0;-50.0]\nGTp[:l109] = [0.0;-100.0]\nGTp[:l110] = [50.0;-100.0]\nGTp[:l111] = [100.0;-100.0]\nGTp[:l112] = [100.0;-50.0]\n\nGTl = Dict{Symbol, Vector{Float64}}()\nGTl[:l1] = [10.0;30]\nGTl[:l2] = [30.0;-30]\nGTl[:l3] = [80.0;40]\nGTl[:l4] = [120.0;-50]NOTE 1. that by using location indicators :l1, :l2, ... or :l100, :l101, ... is of practical benefit when visualizing with existing RoMEPlotting functions.NOTE 2. Landmarks must be in range before range measurements can be made to them."
},

{
    "location": "examples/basic_slamedonut/#Creating-the-Factor-Graph-with-Point2-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Creating the Factor Graph with Point2",
    "category": "section",
    "text": "The first step is to load the required modules, and in our case we will add a few Julia processes to help with the compute later on.  # add more julia processes\nnprocs() < 7 ? addprocs(7-nprocs()) : nothing\n\n# tell Julia that you want to use these modules/namespaces\nusing RoME, DistributionsNOTE Julia uses just-in-time compiling (unless pre-compiled), therefore each time a new function call on a Julia process will be slow, but all following calls to the same functions will be as fast as the statically compiled code.This example exclusively uses Point2 variable node types, which have dimension 2 and represent [x, y] position estimates in the world frame.Next construct the factor graph containing the first pose :l100 (without any knowledge of where it is) and three measured beacons/landmarks :l1,:l2,:l3 – with prior location knowledge for :l1 and :l2:# create the factor graph object\nfg = initfg()\n\n# first pose with no initial estimate\naddNode!(fg, :l100, Point2)\n\n# add three landmarks\naddNode!(fg, :l1, Point2)\naddNode!(fg, :l2, Point2)\naddNode!(fg, :l3, Point2)\n\n# and put priors on :l101 and :l102\naddFactor!(fg, [:l1;], PriorPoint2(MvNormal(GTl[:l1], eye(2))) )\naddFactor!(fg, [:l2;], PriorPoint2(MvNormal(GTl[:l2], eye(2))) )The PriorPoint2 is assumed to be a multivariate normal distribution of covariance eye(2), as well as a weighting factor of [1.0].NOTE API changed to PriorPoint2(::T) where T <: SamplableBelief = PriorPoint2{T} to accept distribution objects and discard (standard in RoME v0.1.5 – see issue 72 here)."
},

{
    "location": "examples/basic_slamedonut/#Adding-Range-Measurements-Between-Variables-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Adding Range Measurements Between Variables",
    "category": "section",
    "text": "Next we connect the three range measurements from the vehicle location :l0 to the three beacons, respectively – and consider that the range measurements are completely relative between the vehicle and beacon position estimates:# first range measurement\nrhoZ1 = norm(GTl[:l1]-GTp[:l100])\nppr = Point2Point2Range( Normal(rhoZ1, 2.0) )\naddFactor!(fg, [:l100;:l101], ppr)\n\n# second range measurement\nrhoZ2 = norm(GTl[:l2]-GTp[:l100])\nppr = Point2Point2Range( Normal(rhoZ2, 3.0) )\naddFactor!(fg, [:l100; :l2], ppr)\n\n# second range measurement\nrhoZ3 = norm(GTl[:l3]-GTp[:l100])\nppr = Point2Point2Range( Normal(rhoZ3, 3.0) )\naddFactor!(fg, [:l100; :l3], ppr)The ranging measurement standard deviation of 2.0 or 3.0 is taken, assuming a Gaussian measurement assumption.   Again, any distribution could have been used. The factor graph should look as follows:writeGraphPdf(fg) # show the factor graph(Image: exranges01)"
},

{
    "location": "examples/basic_slamedonut/#Inference-and-Visualizations-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Inference and Visualizations",
    "category": "section",
    "text": "At this point we can call the solver start interpreting the first results:tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)The factor graph figure above showed the structure between variables and factors. In order to see the numerical values contained in the factor graph, a set of tools are provided by the RoMEPlotting and KernelDensityEstimatePlotting packages. For more details, please see the dedicated visualization discussion here.First look at the two landmark positions :l1, :l2 at (10.0,30),(30.0,-30) respectively.using RoMEPlotting\n\nplotKDE(fg, [:l1;:l2], dims=[1;2], levels=4)(Image: testl1_2)Similarly, the belief estimate for the first vehicle position :l100 is bi-modal, due to the intersection of two range measurements:plotKDE(fg, :l100, dims=[1;2])(Image: testl100)An alternative plotting interface can also be used, that shows a histogram of desired elements instead:drawLandms(fg, from=1, to=101)(Image: testlall)Notice the ring of particles which represents the belief on the third beacon/landmark :l3, which was not constrained by a prior factor. Instead, the belief over the position of :l3 is being estimated simultaneous to estimating the vehicle position :l100."
},

{
    "location": "examples/basic_slamedonut/#Stochastic-Growth-and-Decay-of-Modes-(i.e.-Hypotheses)-1",
    "page": "Singular Ranges-only SLAM (Underdetermined System)",
    "title": "Stochastic Growth and Decay of Modes (i.e. Hypotheses)",
    "category": "section",
    "text": "Next consider the vehicle moving a distance of 50 units–-and by design the direction of travel is not known–-to the next true position. The video above gives away the vehicle position with the cyan line, showing travel in the shape of a lower case \'e\'. Finally, to speed things up, lets write a function that handles the travel (pseudo odometry factors between positions) and ranging measurement factors to beacons.function vehicle_drives_to!(fgl::FactorGraph, pos_sym::Symbol, GTp::Dict, GTl::Dict; measurelimit::R=150.0) where {R <: Real}\n  currvar = union(ls(fgl)...)\n  prev_sym = Symbol(\"l$(maximum(Int[parse(Int,string(currvar[i])[2:end]) for i in 2:length(currvar)]))\")\n  if !(pos_sym in currvar)\n    println(\"Adding variable vertex $pos_sym, not yet in fgl::FactorGraph.\")\n    addNode!(fgl, pos_sym, Point2)\n    @show rho = norm(GTp[prev_sym] - GTp[pos_sym])\n    ppr = Point2Point2Range( Normal(rho, 3.0) )\n    addFactor!(fgl, [prev_sym;pos_sym], ppr)\n  else\n    @warn \"Variable node $pos_sym already in the factor graph.\"\n  end\n  beacons = keys(GTl)\n  for ll in beacons\n    rho = norm(GTl[ll] - GTp[pos_sym])\n    # Check for feasible measurements:  vehicle within 150 units from the beacons/landmarks\n    if rho < measurelimit\n      ppr = Point2Point2Range( Normal(rho, 3.0) )\n      if !(ll in currvar)\n        println(\"Adding variable vertex $ll, not yet in fgl::FactorGraph.\")\n        addNode!(fgl, ll, Point2)\n      end\n      addFactor!(fgl, [pos_sym;ll], ppr)\n    end\n  end\n  nothing\nendAfter pasting (or running) this function in the Julia, a new member definition exists for vehicle_drives_to!.NOTE The exclamation mark at the end of the function name has no syntactic significance in Julia, since the full UTF8 character set is available for functions or variables. Instead, the exclamation serves as a Julia community convention to tell the caller that this function will modify the contents of at least some of the variables being passed into it – in this case the factor graph fg will be modified.Now the actual driving event can be added to the factor graph:#drive to location :l101, then :l102\nvehicle_drives_to!(fg, :l101, GTp, GTl)\nvehicle_drives_to!(fg, :l102, GTp, GTl)\n\n# see the graph\nwriteGraphPdf(fg)NOTE The distance traveled could be any combination of accrued direction and speeds, however, a straight line Gaussian error model is used to keep the visual presentation of this example as simple as possible.The marginal posterior estimates are found by repeating inference over the factor graph, followed drawing all vehicle locations as a contour map:tree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# draw all vehicle locations\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 0:2], dims=[1;2])\n# Gadfly.draw(PDF(\"/tmp/testL100_102.pdf\", 20cm, 10cm),pl) # for storing image to disk\n\npl = plotKDE(fg, [:l3;:l4], dims=[1;2], levels=4)\n# Gadfly.draw(PNG(\"/tmp/testL3_4.png\", 20cm, 10cm),pl)Notice how the vehicle positions have two hypotheses, one left to right and one diagonal right to bottom left – both are valid solutions!(Image: testl100_102)The two \"free\" beacons/landmarks :l3,:l4 still have several modes each, implying insufficient data to constrain either to a strong unimodal belief.(Image: testl3_4)\nvehicle_drives_to!(fg, :l103, GTp, GTl)\nvehicle_drives_to!(fg, :l104, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 0:4], dims=[1;2])\n# Gadfly.draw(PDF(\"/tmp/testL100_104.pdf\", 20cm, 10cm),pl)Moving up to position :l104 still shows strong multiodality in the vehicle position estimates:(Image: testl100_105)vehicle_drives_to!(fg, :l105, GTp, GTl)\nvehicle_drives_to!(fg, :l106, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\nvehicle_drives_to!(fg, :l107, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\nvehicle_drives_to!(fg, :l108, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 2:8], dims=[1;2], levels=6)\n# Gadfly.draw(PDF(\"/tmp/testL103_108.pdf\", 20cm, 10cm),pl)Next we see a strong return to a single dominant mode in all vehicle position estimates, owing to the increased measurements to beacons/landmarks as well as more unimodal estimates in :l3, :l4 beacon/landmark positions.vehicle_drives_to!(fg, :l109, GTp, GTl)\nvehicle_drives_to!(fg, :l110, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n\nvehicle_drives_to!(fg, :l111, GTp, GTl)\nvehicle_drives_to!(fg, :l112, GTp, GTl)\n\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree, N=200)\n\n\npl = plotKDE(fg, [Symbol(\"l$(100+i)\") for i in 7:12], dims=[1;2])\n# Gadfly.draw(PDF(\"/tmp/testL106_112.pdf\", 20cm, 10cm),pl)\n\npl = plotKDE(fg, [:l1;:l2;:l3;:l4], dims=[1;2], levels=4)\n# Gadfly.draw(PDF(\"/tmp/testL1234.pdf\", 20cm, 10cm),pl)\n\npl = drawLandms(fg, from=100)\n# Gadfly.draw(PDF(\"/tmp/testLocsAll.pdf\", 20cm, 10cm),pl)Several location belief estimates exhibit multimodality as the trajectory progresses (not shown), but collapses and finally collapses to a stable set of dominant position estimates.(Image: testl106_112)Landmark estimates are also stable at one estimate:(Image: testl1234)In addition, the SLAM 2D landmark visualization can be re-used to plot more information at once:# pl = drawLandms(fg, from=100, to=200)\n# Gadfly.draw(PDF(\"/tmp/testLocsAll.pdf\", 20cm, 10cm),pl)\n\npl = drawLandms(fg)\n# Gadfly.draw(PDF(\"/tmp/testAll.pdf\", 20cm, 10cm),pl)(Image: testall)This example used the default of N=200 particles per marginal belief. By increasing the number to N=300 throughout the test many more modes and interesting features can be explored, and we refer the reader to an alternative and longer discussion on the same example, in Chapter 6 here."
},

{
    "location": "examples/basic_hexagonal2d/#",
    "page": "Hexagonal 2D SLAM",
    "title": "Hexagonal 2D SLAM",
    "category": "page",
    "text": ""
},

{
    "location": "examples/basic_hexagonal2d/#Hexagonal-2D-SLAM-Example-(Local-Compute)-1",
    "page": "Hexagonal 2D SLAM",
    "title": "Hexagonal 2D SLAM Example (Local Compute)",
    "category": "section",
    "text": "A simple 2D robot trajectory example is expanded below using techniques developed in simultaneous localization and mapping (SLAM). This example is available as a single script here."
},

{
    "location": "examples/basic_hexagonal2d/#Creating-the-Factor-Graph-with-Pose2-1",
    "page": "Hexagonal 2D SLAM",
    "title": "Creating the Factor Graph with Pose2",
    "category": "section",
    "text": "The first step is to load the required modules, and in our case we will add a few Julia processes to help with the compute later on.  # add more julia processes\nnprocs() < 4 ? addprocs(4-nprocs()) : nothing\n\n# tell Julia that you want to use these modules/namespaces\nusing RoME, DistributionsAfter loading the RoME and Distributions modules, we construct a local factor graph object in memory:# start with an empty factor graph object\nfg = initfg()\n\n# Add the first pose :x0\naddNode!(fg, :x0, Pose2)\n\n# Add at a fixed location PriorPose2 to pin :x0 to a starting location\naddFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*eye(3))) )A factor graph object fg (of type ::FactorGraph) has been constructed; the first pose :x0 has been added; and a prior factor setting the origin at [0,0,0] over variable node dimensions [x,y,θ] in the world frame. The type Pose2 is used to indicate what variable is stored in the node. Caesar.jl allows a little more freedom in how factor and variable nodes can be connected, while still allowing for type-assertion to occur.NOTE Julia uses just-in-time compilation (unless pre-compiled)  which is slow the first time a function is called but fast from the second call onwards, since the static function is now cached and ready for use.The next 6 nodes are added with odometry in an counter-clockwise hexagonal manner. Note how variables are denoted with symbols, :x2 == Symbol(\"x2\"):# Drive around in a hexagon\nfor i in 0:5\n  psym = Symbol(\"x$i\")\n  nsym = Symbol(\"x$(i+1)\")\n  addNode!(fg, nsym, Pose2)\n  pp = Pose2Pose2(MvNormal([10.0;0;pi/3], diagm([0.1;0.1;0.1].^2)))\n  addFactor!(fg, [psym;nsym], pp )\nendAt this point it would be good to see what the factor graph actually looks like:writeGraphPdf(fg)You should see the program evince open with this visual:(Image: exfg2d)"
},

{
    "location": "examples/basic_hexagonal2d/#Performing-Inference-1",
    "page": "Hexagonal 2D SLAM",
    "title": "Performing Inference",
    "category": "section",
    "text": "Let\'s run the multimodal-incremental smoothing and mapping (mm-iSAM) solver against this fg object:# perform inference, and remember first runs are slower owing to Julia\'s just-in-time compiling\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n# batchSolve!(fg) # coming soonThis will take a couple of seconds (including first time compiling for all Julia processes)."
},

{
    "location": "examples/basic_hexagonal2d/#Some-Visualization-Plot-1",
    "page": "Hexagonal 2D SLAM",
    "title": "Some Visualization Plot",
    "category": "section",
    "text": "2D plots of the factor graph contents is provided by the RoMEPlotting package. See further discussion on visualizations and packages here.## Inter-operating visualization packages for Caesar/RoME/IncrementalInference exist\nusing RoMEPlotting\n\n# For Juno/Jupyter style use\npl = drawPoses(fg)\n\n# For scripting use-cases you can export the image\nGadfly.draw(Gadfly.PDF(\"/tmp/test.pdf\", 20cm, 10cm),pl)  # or PNG(...)(Image: test)"
},

{
    "location": "examples/basic_hexagonal2d/#Adding-Landmarks-as-Point2-1",
    "page": "Hexagonal 2D SLAM",
    "title": "Adding Landmarks as Point2",
    "category": "section",
    "text": "Suppose some sensor detected a feature of interest with an associated range and bearing measurement The new variable and measurement can be included into the factor graph as follows:# Add landmarks with Bearing range measurements\naddNode!(fg, :l1, Point2, labels=[\"LANDMARK\"])\np2br = Pose2Point2BearingRange(Normal(0,0.1),Normal(20.0,1.0))\naddFactor!(fg, [:x0; :l1], p2br)\n\n# Initialize :l1 numerical values but do not rerun solver\nensureAllInitialized!(fg)NOTE The default behavior for initialization of variable nodes implies the last variable noded added will not have any numerical values yet, please see ContinuousScalar Tutorial for deeper discussion on automatic initialization (autoinit). A slightly expanded plotting function will draw both poses and landmarks (and currently assumes labels starting with :x and :l respectively) – notice the new landmark bottom right:drawPosesLandms(fg)(Image: test)"
},

{
    "location": "examples/basic_hexagonal2d/#One-type-of-Loop-Closure-1",
    "page": "Hexagonal 2D SLAM",
    "title": "One type of Loop-Closure",
    "category": "section",
    "text": "Loop-closures are a major part of SLAM based state estimation. One illustration is to take a second sighting of the same :l1 landmark from the last pose :x6; followed by repeating the inference and re-plotting the result – notice the tighter confidences over all variables:# Add landmarks with Bearing range measurements\np2br2 = Pose2Point2BearingRange(Normal(0,0.1),Normal(20.0,1.0))\naddFactor!(fg, [:x6; :l1], p2br2)\n\n# solve\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# redraw\npl = drawPosesLandms(fg)(Image: test)This concludes the Hexagonal 2D SLAM example."
},

{
    "location": "examples/basic_hexagonal2d/#Interest:-The-Bayes-(Junction)-tree-1",
    "page": "Hexagonal 2D SLAM",
    "title": "Interest: The Bayes (Junction) tree",
    "category": "section",
    "text": "The Bayes (Junction) tree is used as an acyclic (has no loops) computational object, an exact algebraic refactorizating of factor graph, to perform the associated sum-product inference. The visual structure of the tree can extracted by modifying the command tree = wipeBuildNewTree!(fg, drawpdf=true) to produce representations such as this in bt.pdf.(Image: exbt2d)"
},

{
    "location": "examples/interm_fixedlag_hexagonal/#",
    "page": "Fixed-Lag Solving",
    "title": "Fixed-Lag Solving",
    "category": "page",
    "text": ""
},

{
    "location": "examples/interm_fixedlag_hexagonal/#Hexagonal-2D-with-Fixed-Lag-Solving-1",
    "page": "Fixed-Lag Solving",
    "title": "Hexagonal 2D with Fixed-Lag Solving",
    "category": "section",
    "text": "NOTE: This is an experimental feature that is currently being developed. This example provides an overview of how to enable it and the benefits of using fixed-lag solving. The objective is to provide a near-constant solve time for ever-growing graphs by only recalculating the most recent portion. Think of this as a placeholder, as we develop the solution this tutorial will be updated to demonstrate how that is achieved."
},

{
    "location": "examples/interm_fixedlag_hexagonal/#Example-Code-1",
    "page": "Fixed-Lag Solving",
    "title": "Example Code",
    "category": "section",
    "text": "The complete code for this example can be found in the fixed-lag branch of RoME: Hexagonal Fixed-Lag Example."
},

{
    "location": "examples/interm_fixedlag_hexagonal/#Introduction-1",
    "page": "Fixed-Lag Solving",
    "title": "Introduction",
    "category": "section",
    "text": "Fixed-lag solving is enabled when creating the factor-graph. Users provide a window - the quasi fixed-lag constant (QFL) - which defines how many of the most-recent variables should be calculated. Any other variables are \'frozen\'. The objective of this example is to explore providing a near-constant solve time for ever-growing graphs by only recalculating the most recent portion."
},

{
    "location": "examples/interm_fixedlag_hexagonal/#Example-Overview-1",
    "page": "Fixed-Lag Solving",
    "title": "Example Overview",
    "category": "section",
    "text": "In the example, the basic Hexagonal 2D is grown to solve 200 variables. The original example is remains the same, i.e. a vehicle is driving around in a hexagon and seeing the same bearing+range landmark as it crosses the starting point. At every 20th variable, a solve is invoked. Rather than use batchSolve(), the solve is performed in parts (construction of Bayes tree, solving the graph) to get performance statistics as the graph grows.numVariables = 200\nsolveEveryNVariables = 20\nlagLength = 30\n\n# Standard Hexagonal example for totalIterations - solve every iterationsPerSolve iterations.\nfunction runHexagonalExample(fg::FactorGraph, totalIterations::Int, iterationsPerSolve::Int)::DataFrame\n    # Add the first pose :x0\n    addNode!(fg, :x0, Pose2)\n\n    # Add at a fixed location PriorPose2 to pin :x0 to a starting location\n    addFactor!(fg, [:x0], PriorPose2(MvNormal(zeros(3), 0.01*Matrix{Float64}(LinearAlgebra.I, 3,3))))\n\n    # Add a landmark l1\n    addNode!(fg, :l1, Point2, labels=[\"LANDMARK\"])\n\n    # Drive around in a hexagon a number of times\n    solveTimes = DataFrame(GraphSize = [], TimeBuildBayesTree = [], TimeSolveGraph = [])\n    for i in 0:totalIterations\n        psym = Symbol(\"x$i\")\n        nsym = Symbol(\"x$(i+1)\")\n        @info \"Adding pose $nsym...\"\n        addNode!(fg, nsym, Pose2)\n        pp = Pose2Pose2(MvNormal([10.0;0;pi/3], Matrix(Diagonal( [0.1;0.1;0.1].^2 ) )))\n        @info \"Adding odometry factor between $psym -> $nsym...\"\n        addFactor!(fg, [psym;nsym], pp )\n\n        if i % 6 == 0\n            @info \"Creating factor between $psym and l1...\"\n            p2br = Pose2Point2BearingRange(Normal(0,0.1),Normal(20.0,1.0))\n            addFactor!(fg, [psym; :l1], p2br)\n        end\n        if i % iterationsPerSolve == 0 && i != 0\n            @info \"Performing inference!\"\n            if fg.isfixedlag\n                @info \"Quasi fixed-lag is enabled (a feature currently in testing)!\"\n                fifoFreeze!(fg)\n            end\n            tBuild = @timed tree = wipeBuildNewTree!(fg)\n            tInfer = @timed inferOverTree!(fg, tree, N=100)\n            graphSize = length([ls(fg)[1]..., ls(fg)[2]...])\n            push!(solveTimes, (graphSize, tBuild[2], tInfer[2]))\n        end\n    end\n    return solveTimes\nendTwo cases are set up:One solving the full graph every time a solve is performed:# start with an empty factor graph object\nfg = initfg()\n# DO NOT enable fixed-lag operation\nsolverTimesForBatch = runHexagonalExample(fg, numVariables, solveEveryNVariables)The other enabling fixed-lag with a window of 20 variables:fgFixedLag = initfg()\nfgFixedLag.isfixedlag = true\nfgFixedLag.qfl = lagLength\n\nsolverTimesFixedLag = runHexagonalExample(fgFixedLag, numVariables, solveEveryNVariables)The resultant path of the robot can be seen by using RoMEPlotting and is drawn if the visualization lines are uncommented:#### Visualization\n\n# Plot the many iterations to see that it succeeded.\n# Batch\n# drawPosesLandms(fg)\n\n# Fixed lag\n# drawPosesLandms(fgFixedLag)Lastly, the timing results of both scenarios are merged into a single DataFrame table, exported to CSV, and a summary graph is shown using GadFly.using Gadfly\nusing Colors\nusing CSV\n\n# Make a clean dataset\nrename!(solverTimesForBatch, :TimeBuildBayesTree => :Batch_BayedBuild, :TimeSolveGraph => :Batch_SolveGraph);\nrename!(solverTimesFixedLag, :TimeBuildBayesTree => :FixedLag_BayedBuild, :TimeSolveGraph => :FixedLag_SolveGraph);\ntimingMerged = DataFrames.join(solverTimesForBatch, solverTimesFixedLag, on=:GraphSize)\nCSV.write(\"timing_comparison.csv\", timingMerged)\n\nPP = []\npush!(PP, Gadfly.layer(x=timingMerged[:GraphSize], y=timingMerged[:FixedLag_SolveGraph], Geom.path, Theme(default_color=colorant\"green\"))[1]);\npush!(PP, Gadfly.layer(x=timingMerged[:GraphSize], y=timingMerged[:Batch_SolveGraph], Geom.path, Theme(default_color=colorant\"magenta\"))[1]);\n\nplt = Gadfly.plot(PP...,\n    Guide.title(\"Solving Time vs. Iteration for Fixed-Lag Operation\"),\n    Guide.xlabel(\"Solving Iteration\"),\n    Guide.ylabel(\"Solving Time (seconds)\"),\n    Guide.manual_color_key(\"Legend\", [\"fixed\", \"batch\"], [\"green\", \"magenta\"]))\nGadfly.draw(PNG(\"results_comparison.png\", 12cm, 15cm), plt)"
},

{
    "location": "examples/interm_fixedlag_hexagonal/#Results-1",
    "page": "Fixed-Lag Solving",
    "title": "Results",
    "category": "section",
    "text": "Preliminary results for the comparison can be seen below. However, this is just a start and we need to perform more testing. At the moment we are working on providing consistent results and further improving performance/flattening the fixed-lag time. It should be noted that the below graph is not to demonstrate the absolute solve time, but rather the relative behavior of full-graph solve vs. fixed-lag.(Image: Timing comparison of full solve vs. fixed-lag)"
},

{
    "location": "examples/interm_fixedlag_hexagonal/#Additional-Example-1",
    "page": "Fixed-Lag Solving",
    "title": "Additional Example",
    "category": "section",
    "text": "Work In Progress, but In the mean time see the following examples:https://github.com/JuliaRobotics/Caesar.jl/blob/master/examples/wheeled/racecar/apriltagandzed_slam.jl"
},

{
    "location": "examples/basic_definingfactors/#",
    "page": "Creating Custom Variables and Factors",
    "title": "Creating Custom Variables and Factors",
    "category": "page",
    "text": ""
},

{
    "location": "examples/basic_definingfactors/#Defining-New-Variables-and-Factors-1",
    "page": "Creating Custom Variables and Factors",
    "title": "Defining New Variables and Factors",
    "category": "section",
    "text": "TODO: Smooth text and flow."
},

{
    "location": "examples/basic_definingfactors/#Quick-Example-in-One-Dimension-1",
    "page": "Creating Custom Variables and Factors",
    "title": "Quick Example in One Dimension",
    "category": "section",
    "text": "Note these factors already exists in IncrementalInference (and many more in RoME) and are presented here as a first introduction into the process of defining your own factors.User scope Prior, LinearOffset, and MultiModalOffset with arbitrary distributions are defined as:import IncrementalInference: getSample\n\nstruct Prior{T} <: IncrementalInference.FunctorSingleton where T <: Distribution\n  z::T\nend\ngetSample(s::Prior, N::Int=1) = (reshape(rand(s.z,N),1,:), )\nstruct LinearOffset{T} <: IncrementalInference.FunctorPairwise where T <: Distribution\n  z::T\nend\ngetSample(s::LinearOffset, N::Int=1) = (reshape(rand(s.z,N),1,:), )\nfunction (s::LinearOffset)(res::Array{Float64},\n                           userdata::FactorMetadata,\n                           idx::Int,\n                           meas::Tuple{Array{Float64, 2}},\n                           X1::Array{Float64,2},\n                           X2::Array{Float64,2}  )\n  #\n  res[1] = meas[1][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nend\nstruct MultiModalOffset <: IncrementalInference.FunctorPairwise\n  z::Vector{Distribution}\n  c::Categorical\nend\ngetSample(s::MultiModalOffset, N::Int=1) = (reshape.(rand.(s.z, N),1,:)..., rand(s.c, N))\nfunction (s::MultiModalOffset)(res::Array{Float64},\n                               userdata::FactorMetadata,\n                               idx::Int,\n                               meas::Tuple,\n                               X1::Array{Float64,2},\n                               X2::Array{Float64,2}  )\n  #\n  res[1] = meas[meas[end][idx]][idx] - (X2[1,idx] - X1[1,idx])\n  nothing\nendNotice the residual function relating to the two PairwiseFunctor derived definitions. The one dimensional residual functions, res[1] = measurement - prediction, are used during inference to approximate the convolution of conditional beliefs from the sample approximate marginal beliefs of the connected variables."
},

{
    "location": "examples/interm_dynpose/#",
    "page": "Creating DynPose Factor",
    "title": "Creating DynPose Factor",
    "category": "page",
    "text": ""
},

{
    "location": "examples/interm_dynpose/#Adding-Dynamic-Factors-and-Variables-1",
    "page": "Creating DynPose Factor",
    "title": "Adding Dynamic Factors and Variables",
    "category": "section",
    "text": "This tutorial describes how a new factor can be developed, beyond the pre-existing implementation in RoME.jl.  Factors can accept any number of variable dependencies and allow for a wide class of allowable function calls can be used.  Our intention is to make it as easy as possible for users to create their own factor types."
},

{
    "location": "examples/interm_dynpose/#Example:-Adding-Velocity-to-RoME.Point2-1",
    "page": "Creating DynPose Factor",
    "title": "Example: Adding Velocity to RoME.Point2",
    "category": "section",
    "text": "A smaller example in two dimensions where we wish to estimate the velocity of some target:  Consider two variables :x0 with a prior as well as a conditional–-likelihood for short–-to variable :x1.  Priors are in the \"global\" reference frame (how ever you choose to define it), while likelihoods are in the \"local\" / \"relative\" frame that only exist between variables.(Image: dynpoint2fg)"
},

{
    "location": "examples/interm_dynpose/#Brief-on-Variable-Node-softtypes-1",
    "page": "Creating DynPose Factor",
    "title": "Brief on Variable Node softtypes",
    "category": "section",
    "text": "Variable nodes retain meta data (so called \"soft types\") describing the type of variable.  Common VariableNode types are RoME.Point2D, RoME.Pose3D.  VariableNode soft types are passed during construction of the factor graph, for example:v1 = addNode!(fg, :x1, Pose2)Certain cases require that more information be retained for each VariableNode, and velocity calculations are a clear example where time stamp data across positions is required.  Note Larger data can also be stored under the bigdata framework which is discussed here (TBD).If the required VariableNode does not exist, then one can be created, such as adding velocity states with DynPoint2:mutable struct DynPoint2 <: IncrementalInference.InferenceVariable\n  ut::Int64 # microsecond time\n  dims::Int\n  labels::Vector{String}\n  DynPoint2(;ut::Int64=0, labels::Vector{<:AbstractString}=String[]) = new(ut, 4, labels)\nendThe dims field is permanently set to 4, i.e. [x, y, dx/dt, dy/dt].  Labels represent special labels that can be used for more efficient indexing in database systems.  The utparameter is for storing the microsecond time stamp for that variable node.In order to implement your own factor type outside IncrementalInference you should import the required identifiers, as follows:using IncrementalInference\nimport IncrementalInference: getSampleNote that new factor types can be defined at any time, even after you have started to construct the FactorGraph object."
},

{
    "location": "examples/interm_dynpose/#DynPoint2VelocityPrior-1",
    "page": "Creating DynPose Factor",
    "title": "DynPoint2VelocityPrior",
    "category": "section",
    "text": "Work in progress.mutable struct DynPoint2VelocityPrior{T} <: IncrementalInference.FunctorSingleton where {T <: Distribution}\n  z::T\n  DynPoint2VelocityPrior{T}() where {T <: Distribution} = new{T}()\n  DynPoint2VelocityPrior(z1::T) where {T <: Distribution} = new{T}(z1)\nend\ngetSample(dp2v::DynPoint2VelocityPrior, N::Int=1) = (rand(dp2v.z,N), )"
},

{
    "location": "examples/interm_dynpose/#DynPoint2DynPoint2-(preintegration)-1",
    "page": "Creating DynPose Factor",
    "title": "DynPoint2DynPoint2 (preintegration)",
    "category": "section",
    "text": "The basic idea is that change in position is composed of three components (originating from double integration of Newton\'s second law):(Image: deltapositionplus) ( eq. 1)DynPoint2DynPoint2 factor is using the above equation to define the difference in position between the two DynPoint2s.  The position part stored in DynPoint2DynPoint2 factor corresponds to (Image: deltaposplusonly).  A new multi-variable (so called \"pairwise\") factor between any number of variables is defined with three elements:Factor type definition that inherits either IncrementalInference.FunctorPairwise or IncrementalInference.FunctorPairwiseMinimize;mutable struct DynPoint2DynPoint2{T} <: IncrementalInference.FunctorPairwise where {T <: Distribution}\n  z::T\n  DynPoint2DynPoint2{T}() where {T <: Distribution} = new{T}()\n  DynPoint2DynPoint2(z1::T) where {T <: Distribution} = new{T}(z1)\nendA sampling function with exactly the signature: getSample(dp2dp2::DynPoint2DynPoint2, N::Int=1) and returning a Tuple (legacy reasons);getSample(dp2dp2::DynPoint2DynPoint2, N::Int=1) = (rand(dp2dp2.z,N), )A residual or minimization function with exactly the signature described below.Residual (related to FunctorPairwise) or factor minimization function (related to FunctorPairwiseMinimize) signatures should match this dp2dp2::DynPoint2DynPoint2 example:function (dp2dp2::DynPoint2DynPoint2)(\n            res::Array{Float64},\n            userdata,\n            idx::Int,\n            meas::Tuple,\n            Xs...  )::Nothingwhere Xs can be expanded to the particular number of variable nodes this factor will be associated, and note they are order sensitive at addFactor!(fg, ...) time.  The res parameter is a vector of the same dimension defined by the largest of the Xs terms.  The userdata value contains the small metadata / userdata portions of information that was introduced to the factor graph at construction time – please consult error(string(fieldnames(userdata))) for details at this time.  This is a relatively new feature in the code and likely to be improved.  The idx parameter represents a legacy index into the measurement meas[1] and variables Xs to select the desired marginal sample value.  Future versions of the code plan to remove the idx parameter entirely.  The Xs array of parameter are each of type ::Array{Float64,2} and contain the estimated samples from each of the current best marginal belief estimates of the factor graph variable node.  function (dp2dp2::DynPoint2DynPoint2)(\n            res::Array{Float64},\n            userdata,\n            idx::Int,\n            meas::Tuple,\n            Xi::Array{Float64,2},\n            Xj::Array{Float64,2}  )\n  #\n  z = meas[1][:,idx]\n  xi, xj = Xi[:,idx], Xj[:,idx]\n  dt = (userdata.variableuserdata[2].ut - userdata.variableuserdata[1].ut)*1e-6   # roughly the intended use of userdata\n  res[1:2] = z[1:2] - (xj[1:2] - (xi[1:2]+dt*xi[3:4]))\n  res[3:4] = z[3:4] - (xj[3:4] - xi[3:4])\n  nothing\nendA brief usage example looks as follows, and further questions about how the preintegration strategy was implemented can be traced through the original issue JuliaRobotics/RoME.jl#60 or the literature associated with this project, or contact for more information.using RoME, Distributions\nfg = initfg()\nv0 = addNode!(fg, :x0, DynPoint2(ut=0))\n\n# Prior factor as boundary condition\npp0 = DynPoint2VelocityPrior(MvNormal([zeros(2);10*ones(2)], 0.1*eye(4)))\nf0 = addFactor!(fg, [:x0;], pp0)\n\n# conditional likelihood between Dynamic Point2\nv1 = addNode!(fg, :x1, DynPoint2(ut=1000_000)) # time in microseconds\ndp2dp2 = DynPoint2DynPoint2(MvNormal([10*ones(2);zeros(2)], 0.1*eye(4)))\nf1 = addFactor!(fg, [:x0;:x1], dp2dp2)\n\nensureAllInitialized!(fg)\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\nusing KernelDensityEstimate\n@show x0 = getKDEMax(getVertKDE(fg, :x0))\n# julia> ... = [-0.19441, 0.0187019, 10.0082, 10.0901]\n@show x1 = getKDEMax(getVertKDE(fg, :x1))\n # julia> ... = [19.9072, 19.9765, 10.0418, 10.0797]"
},

{
    "location": "examples/interm_dynpose/#VelPoint2VelPoint2-(back-differentiation)-1",
    "page": "Creating DynPose Factor",
    "title": "VelPoint2VelPoint2 (back-differentiation)",
    "category": "section",
    "text": "In case the preintegrated approach is not the first choice, we include VelPoint2VelPoint2 <: IncrementalInference.FunctorPairwiseMinimize as a second likelihood factor example which may seem more intuitive:mutable struct VelPoint2VelPoint2{T} <: IncrementalInference.FunctorPairwiseMinimize where {T <: Distribution}\n  z::T\n  VelPoint2VelPoint2{T}() where {T <: Distribution} = new{T}()\n  VelPoint2VelPoint2(z1::T) where {T <: Distribution} = new{T}(z1)\nend\ngetSample(vp2vp2::VelPoint2VelPoint2, N::Int=1) = (rand(vp2vp2.z,N), )\nfunction (vp2vp2::VelPoint2VelPoint2)(\n                res::Array{Float64},\n                userdata,\n                idx::Int,\n                meas::Tuple,\n                Xi::Array{Float64,2},\n                Xj::Array{Float64,2}  )\n  #\n  z = meas[1][:,idx]\n  xi, xj = Xi[:,idx], Xj[:,idx]\n  dt = (userdata.variableuserdata[2].ut - userdata.variableuserdata[1].ut)*1e-6   # roughly the intended use of userdata\n  dp = (xj[1:2]-xi[1:2])\n  dv = (xj[3:4]-xi[3:4])\n  res[1] = 0.0\n  res[1] += sum((z[1:2] - dp).^2)\n  res[1] += sum((z[3:4] - dv).^2)\n  res[1] += sum((dp/dt - xi[3:4]).^2)  # (dp/dt - 0.5*(xj[3:4]+xi[3:4])) # midpoint integration\n  res[1]\nendA similar usage example here shows:fg = initfg()\n\n# add three point locations\nv0 = addNode!(fg, :x0, DynPoint2(ut=0))\nv1 = addNode!(fg, :x1, DynPoint2(ut=1000_000))\nv2 = addNode!(fg, :x2, DynPoint2(ut=2000_000))\n\n# Prior factor as boundary condition\npp0 = DynPoint2VelocityPrior(MvNormal([zeros(2);10*ones(2)], 0.1*eye(4)))\nf0 = addFactor!(fg, [:x0;], pp0)\n\n# conditional likelihood between Dynamic Point2\ndp2dp2 = VelPoint2VelPoint2(MvNormal([10*ones(2);zeros(2)], 0.1*eye(4)))\nf1 = addFactor!(fg, [:x0;:x1], dp2dp2)\n\n# conditional likelihood between Dynamic Point2\ndp2dp2 = VelPoint2VelPoint2(MvNormal([10*ones(2);zeros(2)], 0.1*eye(4)))\nf2 = addFactor!(fg, [:x1;:x2], dp2dp2)\n\n# Graphs.plot(fg.g)\nensureAllInitialized!(fg)\ntree = wipeBuildNewTree!(fg)\ninferOverTree!(fg, tree)\n\n# see the output\n@show x0 = getKDEMax(getVertKDE(fg, :x0))\n@show x1 = getKDEMax(getVertKDE(fg, :x1))\n@show x2 = getKDEMax(getVertKDE(fg, :x2))Producing output:x0 = getKDEMax(getVertKDE(fg, :x0)) = [0.101503, -0.0273216, 9.86718, 9.91146]\nx1 = getKDEMax(getVertKDE(fg, :x1)) = [10.0087, 9.95139, 10.0622, 10.0195]\nx2 = getKDEMax(getVertKDE(fg, :x2)) = [19.9381, 19.9791, 10.0056, 9.92442]"
},

{
    "location": "examples/interm_dynpose/#IncrementalInference.jl-Defining-Factors-(Future-API)-1",
    "page": "Creating DynPose Factor",
    "title": "IncrementalInference.jl Defining Factors (Future API)",
    "category": "section",
    "text": "We would like to remove the idx indexing from the residual function calls, since that is an unnecessary burden on the user.  Instead, the package will use views and SubArray types to simplify the interface.  Please contact author for more details (8 June 2018)."
},

{
    "location": "examples/interm_dynpose/#Contributions-1",
    "page": "Creating DynPose Factor",
    "title": "Contributions",
    "category": "section",
    "text": "Thanks to mc2922 for raising the catalyst issue and conversations that followed from JuliaRobotics/RoME.jl#60."
},

{
    "location": "func_ref/#",
    "page": "Function Reference",
    "title": "Function Reference",
    "category": "page",
    "text": ""
},

{
    "location": "func_ref/#Function-Reference-1",
    "page": "Function Reference",
    "title": "Function Reference",
    "category": "section",
    "text": "Pages = [\n    \"func_ref.md\"\n]\nDepth = 3"
},

{
    "location": "func_ref/#Caesar.getPoseExVertexNeoIDs",
    "page": "Function Reference",
    "title": "Caesar.getPoseExVertexNeoIDs",
    "category": "function",
    "text": "getPoseExVertexNeoIDs(conn; ready, backendset, session, reqbackendset)\n\n\nReturn array of tuples with ExVertex IDs and Neo4j IDs for all poses.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getLandmOtherSessNeoIDs",
    "page": "Function Reference",
    "title": "Caesar.getLandmOtherSessNeoIDs",
    "category": "function",
    "text": "getLandmOtherSessNeoIDs{T <: AbstractString}(::CloudGraph, session::T=\"\", robot::T=\"\", user::T=\"\", multisessions=Vector{T}())\n\nReturn dict of dict of Neo4j vertex IDs by session and landmark symbols.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.db2jld",
    "page": "Function Reference",
    "title": "Caesar.db2jld",
    "category": "function",
    "text": "db2jld(cgl::CloudGraph, session::AbstractString, filename::AbstractString)\n\nFetch and save a FactorGraph session to a jld, using CloudGraph object and session definition.\n\n\n\n\n\ndb2jld(filename::AbstractString; addrdict::NothingUnion{Dict{AbstractString, AbstractString}}=nothing )\n\nFetch and save a FactorGraph session to a jld, using or asking STDIN for credentials in the addrdict field.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.fetchrobotdatafirstpose",
    "page": "Function Reference",
    "title": "Caesar.fetchrobotdatafirstpose",
    "category": "function",
    "text": "fetchrobotdatafirstpose(cg::CloudGraph, session::AbstractString, robot::AbstractString, user::AbstractString)\n\nReturn dict of JSON parsed \"robot_description\" field as was inserted by counterpart insertrobotdatafirstpose! function. Used for storing general robot specific data in easily accessible manner.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.executeQuery",
    "page": "Function Reference",
    "title": "Caesar.executeQuery",
    "category": "function",
    "text": "executeQuery(connection, query)\n\n\nRun Neo4j Cypher queries on the cloudGraph database, and return Tuple with the unparsed (results, loadresponse).\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.appendvertbigdata!",
    "page": "Function Reference",
    "title": "Caesar.appendvertbigdata!",
    "category": "function",
    "text": "appendvertbigdata!(cloudGraph, cv, description, data)\n\n\nAppend big data element into current blob store and update associated global vertex information.\n\n\n\n\n\nappendvertbigdata!(fgl, vert, description, data)\n\n\nAppend big data element into current blob store and update associated global vertex information.\n\n\n\n\n\nappendvertbigdata!(fg, sym, descr, data)\n\nAppend big data element into current blob store using parent appendvertbigdata!, but here specified by symbol of variable node in the FactorGraph. Note the default data layer api definition. User must define dlapi to refetching the  vertex from the data layer. localapi avoids repeated network database fetches.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.updatenewverts!",
    "page": "Function Reference",
    "title": "Caesar.updatenewverts!",
    "category": "function",
    "text": "updatenewverts!(fgl::FactorGraph; N::Int)\n\nConvert vertices of session in Neo4j DB with Caesar.jl\'s required data elements in preparation for MM-iSAMCloudSolve process.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getVertNeoIDs!",
    "page": "Function Reference",
    "title": "Caesar.getVertNeoIDs!",
    "category": "function",
    "text": "getVertNeoIDs!(::CloudGraph, res::Dict{Symbol, Int}; session::AbstractString=\"NA\", robot::AbstractString=\"NA\", user::AbstractString=\"NA\")\n\nInsert into and return dict res with Neo4j IDs of ExVertex labels as stored per session in Neo4j database.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.whosNear3D",
    "page": "Function Reference",
    "title": "Caesar.whosNear3D",
    "category": "function",
    "text": "whosNear3D(cg, session, robot, user; x, y, z, roll, pitch, yaw, dist, angle)\n\n\nFind vertices near the point specified and return dictionary of symbol to Neo4j ID pairs.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getprpt2kde",
    "page": "Function Reference",
    "title": "Caesar.getprpt2kde",
    "category": "function",
    "text": "getprp2kde(::CloudGraph, neoids::Vector{Int}; N::Int=100)\n\nReturn PriorPoint2DensityNH with N points based on beliefs of neoids, and equal share null hypothesis between length(neoids)+1 beliefs.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.whosNear2D",
    "page": "Function Reference",
    "title": "Caesar.whosNear2D",
    "category": "function",
    "text": "whosNear2D(cg, session, robot, user; x, y, yaw, dist, angle)\n\n\nFind vertices near the point specified and return dictionary of symbol to Neo4j ID pairs.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.hasBigDataElement",
    "page": "Function Reference",
    "title": "Caesar.hasBigDataElement",
    "category": "function",
    "text": "hasBigDataElement(vertex, description)\n\n\nReturn true if vertex has bigDataElements with matching description.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getAllLandmarkNeoIDs",
    "page": "Function Reference",
    "title": "Caesar.getAllLandmarkNeoIDs",
    "category": "function",
    "text": "getAllLandmarkNeoIDs(::Dict{Symbol, Dict{Symbol, Int}}, ::Symbol)\n\nReturn Vector{Int} of Neo4j vertex IDs relating to symbol, as listed in lm2others.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.consoleaskuserfordb",
    "page": "Function Reference",
    "title": "Caesar.consoleaskuserfordb",
    "category": "function",
    "text": "consoleaskuserfordb(; nparticles, drawdepth, clearslamindb, multisession, drawedges)\n\n\nObtain database addresses and login credientials from STDIN, as well as a few case dependent options.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.standardcloudgraphsetup",
    "page": "Function Reference",
    "title": "Caesar.standardcloudgraphsetup",
    "category": "function",
    "text": "standardcloudgraphsetup(; addrdict, nparticles, drawdepth, drawedges, clearslamindb, multisession)\n\n\nConnect to databases via network according to addrdict, or ask user for credentials and return active cloudGraph object, as well as addrdict.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.resetentireremotesession",
    "page": "Function Reference",
    "title": "Caesar.resetentireremotesession",
    "category": "function",
    "text": "resetentireremotesession(conn, session, robot, user)\n\nmatch (n:session) remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width set n :NEWDATA return n\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getfirstpose",
    "page": "Function Reference",
    "title": "Caesar.getfirstpose",
    "category": "function",
    "text": "getfirstpose(cg::CloudGraph, session::AbstractString, robot::AbstractString, user::AbstractString)\n\nReturn Tuple{Symbol, Int} of first pose symbol and Neo4j node ID.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getExVertexNeoIDs",
    "page": "Function Reference",
    "title": "Caesar.getExVertexNeoIDs",
    "category": "function",
    "text": "getExVertexNeoIDs(conn; label, ready, backendset, session, robot, user, reqbackendset)\n\n\nReturn array of tuples with ExVertex IDs and Neo4j IDs for vertices with label in session.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.findExistingMSConstraints",
    "page": "Function Reference",
    "title": "Caesar.findExistingMSConstraints",
    "category": "function",
    "text": "findExistingMSConstraints(fgl::FactorGraph)\n\nReturn Dict{Symbol, Int} of vertex symbol to Neo4j node ID of MULTISESSION constraints in this fgl.sessionname.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.removeNeo4jID",
    "page": "Function Reference",
    "title": "Caesar.removeNeo4jID",
    "category": "function",
    "text": "removeNeo4jID(cg::CloudGraph, neoid=-1)\n\nRemove node from Neo4j according to Neo4j Node ID. Big data elements that may be associated with this node are not removed.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getBigDataElement",
    "page": "Function Reference",
    "title": "Caesar.getBigDataElement",
    "category": "function",
    "text": "getBigDataElement(vertex, description)\n\n\nWalk through vertex bigDataElements and return the last matching description.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.rmInstMultisessionPriors!",
    "page": "Function Reference",
    "title": "Caesar.rmInstMultisessionPriors!",
    "category": "function",
    "text": "rmInstMultisessionPriors!(::CloudGraph; session<:AbstractString=, multisessions::Vector{<:AbstractString}= )\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.insertrobotdatafirstpose!",
    "page": "Function Reference",
    "title": "Caesar.insertrobotdatafirstpose!",
    "category": "function",
    "text": "insertrobotdatafirstpose!(cg::CloudGraph, session::AbstractString, robot::AbstractString, user::AbstractString, robotdict::Dict)\n\nSaves robotdict via JSON to first pose in a SESSION in the database. Used for storing general robot specific data in easily accessible manner. Can fetch later retrieve same dict with counterpart fetchrobotdatafirstpose function.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.fetchsubgraph!",
    "page": "Function Reference",
    "title": "Caesar.fetchsubgraph!",
    "category": "function",
    "text": "fetchsubgraph!(::FactorGraph, ::Vector{CloudVertex}, numneighbors::Int=0)\n\nFetch and insert list of CloudVertices into FactorGraph object, up to neighbor depth.\n\n\n\n\n\nfetchsubgraph!(::FactorGraph, ::Vector{Int}, numneighbors::Int=0)\n\nFetch and insert list of Neo4j IDs into FactorGraph object, up to neighbor depth.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getLocalSubGraphMultisession",
    "page": "Function Reference",
    "title": "Caesar.getLocalSubGraphMultisession",
    "category": "function",
    "text": "getLocalSubGraphMultisession{T <: AbstractString}(cg::CloudGraph, lm2others; session::T=\"\", numneighbors::Int=0)\n\nReturn subgraph copy of type FactorGraph contaning values from session in lm2others, and Vector{Symbol} of primary key symbols used for graph exstraction.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar.getnewvertdict",
    "page": "Function Reference",
    "title": "Caesar.getnewvertdict",
    "category": "function",
    "text": "getnewvertdict(conn, session::AbstractString, robot::AbstractString, user::AbstractString)\n\nReturn a dictionary with frtend and mongo_keys json string information for :NEWDATA elements in Neo4j database.\n\n\n\n\n\n"
},

{
    "location": "func_ref/#Caesar-1",
    "page": "Function Reference",
    "title": "Caesar",
    "category": "section",
    "text": "getPoseExVertexNeoIDs\ngetLandmOtherSessNeoIDs\ndb2jld\nfetchrobotdatafirstpose\nexecuteQuery\nappendvertbigdata!\nupdatenewverts!\ngetVertNeoIDs!\nwhosNear3D\ngetprpt2kde\nwhosNear2D\nhasBigDataElement\ngetAllLandmarkNeoIDs\nconsoleaskuserfordb\nstandardcloudgraphsetup\nresetentireremotesession\ngetfirstpose\ngetExVertexNeoIDs\nfindExistingMSConstraints\nremoveNeo4jID\ngetBigDataElement\nrmInstMultisessionPriors!\ninsertrobotdatafirstpose!\nfetchsubgraph!\ngetLocalSubGraphMultisession\ngetnewvertdict"
},

]}
