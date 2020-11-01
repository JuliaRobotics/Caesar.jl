using Documenter, Caesar

import IncrementalInference: fmcmc!, localProduct, productpartials!, prodmultiplefullpartials, prodmultipleonefullpartials, setfreeze!
import IncrementalInference: cliqGibbs, packFromLocalPotentials!, treeProductDwn, updateFGBT!, upGibbsCliqueDensity
import IncrementalInference: initfg, downGibbsCliqueDensity

using KernelDensityEstimatePlotting
# import KernelDensityEstimatePlotting: plotKDE
using RoMEPlotting

# until namespaces are properly figured out
using DistributedFactorGraphs
import DistributedFactorGraphs: showFactor, showVariable

makedocs(
    modules = [Caesar, RoME, IncrementalInference, RoMEPlotting, KernelDensityEstimatePlotting],
    format = Documenter.HTML(),
    sitename = "Caesar.jl",
    pages = Any[
        "Introduction" => "index.md",
        "Welcome" => [
            "Installation" => "installation_environment.md",
            "FAQ" => "faq.md",
        ],
        "Getting Started" => [
            "Initial Concepts" => "concepts/concepts.md",
            "Building Factor Graphs" => "concepts/building_graphs.md",
            "Solving and Interacting" => "concepts/interacting_fgs.md",
            "Internal Variables/Factors" => "concepts/available_varfacs.md",
            "Multi-Modal/Hypothesis" => "concepts/dataassociation.md",
            "Flux (NN) Factors" => "concepts/flux_factors.md",
            "Plotting (2D)" => "concepts/2d_plotting.md",
            "Entry=>Data Blob" => "concepts/entry_data.md",
            "Multi-Language Support" => "concepts/multilang.md",
            "Cloud Server/Database" => "concepts/database_interactions.md",
            "Multi-session/agent Solving" => "concepts/multisession.md",
            "Visualization (3D)" => "concepts/arena_visualizations.md",
        ],
        "Examples" => [
            "Caesar Examples" => "examples/examples.md",
            "ContinuousScalar as 1D Example" => "examples/basic_continuousscalar.md",
            "Under-defined Trilateration, 2D" => "examples/basic_slamedonut.md",
            "Hexagonal 2D SLAM" => "examples/basic_hexagonal2d.md",
            "Fixed-Lag Solving 2D" => "examples/interm_fixedlag_hexagonal.md",
            "Dead Reckon Tether" => "examples/deadreckontether.md",
        ],
        "Principles" => [
            "Filters vs. Graphs" => "principles/filterCorrespondence.md",
            "Generic Convolutions" => "principles/approxConvDensities.md",
            "Multiplying Functions (.py)" => "principles/multiplyingDensities.md",
            "Bayes (Junction) tree" => "principles/bayestreePrinciples.md",
            "Advanced Bayes Tree Topics" => "principles/initializingOnBayesTree.md",
            "Multimodal iSAM Algorithm" => "concepts/mmisam_alg.md",
        ],
        "How to Expand?" => [
            "Custom Variables and Factors" => "concepts/adding_variables_factors.md",
            "Creating Variables and Factors" => "examples/basic_definingfactors.md",
            "Creating DynPose Factor" => "examples/interm_dynpose.md"
        ],
        "Developer Zone" => [
            "Wiki Pointers" => "dev/wiki.md"
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
