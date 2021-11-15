using Documenter, Caesar
using RoME

import IncrementalInference: fmcmc!, localProduct, prodmultiplefullpartials, prodmultipleonefullpartials, setfreeze!
import IncrementalInference: cliqGibbs, packFromLocalPotentials!, treeProductDwn, updateFGBT!, upGibbsCliqueDensity
import IncrementalInference: initfg, downGibbsCliqueDensity
import IncrementalInference: solveGraphParametric, solveGraphParametric!

using KernelDensityEstimatePlotting
# import KernelDensityEstimatePlotting: plotKDE
using RoMEPlotting

# until namespaces are properly figured out
using DistributedFactorGraphs
import DistributedFactorGraphs: showFactor, showVariable
import DistributedFactorGraphs: deleteVariable!

makedocs(
    modules = [Caesar, RoME, IncrementalInference, RoMEPlotting, KernelDensityEstimatePlotting, DistributedFactorGraphs],
    format = Documenter.HTML(),
    sitename = "Caesar.jl",
    pages = Any[
        "Welcome" => "index.md",
        "Introduction" => [
            "Introduction" => "introduction.md",
            "Gaussian vs. Non-Gaussian" => "concepts/why_nongaussian.md",
            "Installation" => "installation_environment.md",
            "Using Julia" => "concepts/using_julia.md",
            "FAQ" => "faq.md",
        ],
        "Getting Started" => [
            "Initial Concepts" => "concepts/concepts.md",
            "Building Graphs" => "concepts/building_graphs.md",
            "Solving Graphs" => "concepts/solving_graphs.md",
            "Interact w Graphs" => "concepts/interacting_fgs.md",
            "Multi-Modal/Hypothesis" => "concepts/dataassociation.md",
            "Parallel Processing" => "concepts/parallel_processing.md",
            "[DEV] Parametric Solve" => "examples/parametric_solve.md",
        ],
        "Examples" => [
            "Caesar Examples" => "examples/examples.md",
            "Canonical 1D Example" => "examples/basic_continuousscalar.md",
            "Underconstrained Range-only" => "examples/basic_slamedonut.md",
            "Hexagonal 2D SLAM" => "examples/basic_hexagonal2d.md",
            "Fixed-Lag Solving 2D" => "examples/interm_fixedlag_hexagonal.md",
            "Dead Reckon Tether" => "examples/deadreckontether.md",
        ],
        "Graph Library" => [
            "Canonical Generators" => "examples/canonical_graphs.md",
            "Entry=>Data Blob" => "concepts/entry_data.md",
            "Variables/Factors" => "concepts/available_varfacs.md",
            "Flux (NN) Factors" => "concepts/flux_factors.md",
            "Images and AprilTags" => "examples/using_images.md",
        ],
        "Visualization" => [
            "Installing Viz" => "install_viz.md",
            "Plotting (2D)" => "concepts/2d_plotting.md",
            "Visualization (3D)" => "concepts/arena_visualizations.md",            
        ],
        "Middlewares" => [
            "ROS Middleware" => "examples/using_ros.md",
            "Compile Binaries" => "concepts/compile_binary.md",
            "Zero Install Solution" => "concepts/zero_install.md",
            "Multi-session/agent Solving" => "concepts/multisession.md",
            "Multi-Language Support" => "concepts/multilang.md",
        ],
        "How to Expand?" => [
            "Pkg Framework" => "caesar_framework.md",
            "Custom Variables and Factors" => "examples/adding_variables_factors.md",
            "Creating Variables" => "examples/custom_variables.md",
            "Creating Factors" => "examples/basic_definingfactors.md",
            "More Functions" => "func_ref.md",
        ],
        "Principles" => [
            "Filters vs. Graphs" => "principles/filterCorrespondence.md",
            "Generic Convolutions" => "principles/approxConvDensities.md",
            "Multiplying Functions (.py)" => "principles/multiplyingDensities.md",
            "Bayes (Junction) tree" => "principles/bayestreePrinciples.md",
            "Advanced Bayes Tree Topics" => "principles/initializingOnBayesTree.md",
            "Non-Gaussian Algorithm" => "concepts/mmisam_alg.md",
        ],
        "Developer Zone" => [
            "Wiki Pointers" => "dev/wiki.md",
            "Legacy Factors" => "examples/legacy_deffactors.md",
            "Creating DynPose Factor" => "principles/interm_dynpose.md",
            "Known Issue List" => "dev/known_issues.md",
            "Internal Functions" => "dev/internal_fncs.md",
            ],
        "Literature" => [
            "References" => "refs/literature.md"
        ],
    ]
    # html_prettyurls = !("local" in ARGS),
    )


deploydocs(
    repo   = "github.com/JuliaRobotics/Caesar.jl.git",
    target = "build"
)
