using Documenter, Caesar

import IncrementalInference: fmcmc!, localProduct, productpartials!, prodmultiplefullpartials, prodmultipleonefullpartials, setfreeze!
import IncrementalInference: cliqGibbs, downMsgPassingRecursive, packFromLocalPotentials!, treeProductDwn, updateFGBT!, upGibbsCliqueDensity
import IncrementalInference: initfg, downGibbsCliqueDensity

using KernelDensityEstimatePlotting
# import KernelDensityEstimatePlotting: plotKDE
using RoMEPlotting

# until namespaces are properly figured out
using DistributedFactorGraphs
import DistributedFactorGraphs: showFactor

makedocs(
    modules = [Caesar, RoME, IncrementalInference, RoMEPlotting, KernelDensityEstimatePlotting],
    format = Documenter.HTML(),
    sitename = "Caesar.jl",
    pages = Any[
        "Home" => "index.md",
        "Getting Started" => [
            "Installation" => "installation_environment.md",
            "FAQ" => "faq.md",
        ],
        "Initial Concepts" => [
            "Caesar Concepts" => "concepts/concepts.md",
            "Building Factor Graphs" => "concepts/building_graphs.md",
            "Available Variables/Factors" => "concepts/available_varfacs.md",
            "Interacting w/ Factor Graphs" => "concepts/interacting_fgs.md",
            "Multi-Language Support" => "concepts/multilang.md",
            "Arena Visualization" => "concepts/arena_visualizations.md",
            "Cloud Server/Database" => "concepts/database_interactions.md",
            "Multi/Cross Session Solving" => "concepts/multisession.md",
        ],
        "Examples" => [
            "Caesar Examples" => "examples/examples.md",
            "ContinuousScalar as 1D Example" => "examples/basic_continuousscalar.md",
            "Under-defined Trilateration SLAM 2D" => "examples/basic_slamedonut.md",
            "Hexagonal 2D SLAM" => "examples/basic_hexagonal2d.md",
            "Fixed-Lag Solving 2D" => "examples/interm_fixedlag_hexagonal.md",
        ],
        "Principles" => [
            "Multiplying Functions (.py)" => "principles/multiplyingDensities.md",
            "Generic Convolutions" => "principles/approxConvDensities.md",
            "Filters vs. Graphs" => "principles/filterCorrespondence.md",
            "Bayes (Junction) tree" => "principles/bayestreePrinciples.md",
            "Multimodal iSAM Algorithm" => "concepts/mmisam_alg.md",
            "Advanced Bayes Tree Topics" => "principles/initializingOnBayesTree.md",
        ],
        "How to Expand?" => [
            "Custom Variables and Factors" => "concepts/adding_variables_factors.md",
            "Creating Variables and Factors" => "examples/basic_definingfactors.md",
            "Creating DynPose Factor" => "examples/interm_dynpose.md"
        ],
        "Developer Zone" => [
            "Wiki Pointer" => "dev/wiki.md"
        ],
        "Literature" => [
            "References" => "refs/literature.md"
        ],
        "Function Reference" => [
            "Caesar's Reference" => "func_ref.md",
            "Visualization Reference" => "vis_func_ref.md",
        ]
    ]
    # html_prettyurls = !("local" in ARGS),
    )


deploydocs(
    repo   = "github.com/JuliaRobotics/Caesar.jl.git",
    target = "build"
)
