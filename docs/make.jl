using Documenter, Caesar

makedocs(
    modules = [Caesar],
    format = :html,
    sitename = "Caesar.jl",
    pages = Any[
        "Home" => "index.md",
        "Getting Started" => [
            "Installation" => "installation_environment.md"
        ],
        "Concepts" => [
            "Caesar Concepts" => "concepts/concepts.md",
            "Building Factor Graphs" => "concepts/building_graphs.md",
            "Arena Visualization" => "concepts/arena_visualizations.md",
            "Using Caesar's Multi-Language Support" => "concepts/zmq.md",
            "Adding New Variables and Factors" => "concepts/adding_variables_factors.md",
            "Using Caesar Database Operation" => "concepts/database_interactions.md"
        ],
        "Examples" => [
            "Caesar Examples" => "examples/examples.md",
            "Basics: Hexagonal 2D SLAM" => "examples/basic_hexagonal2d.md",
            "Basics: Singular Ranges-only SLAM" => "examples/basic_slamedonut.md",
            "Basics: ContinuousScalar" => "examples/basic_continuousscalar.md",
            "Basics: Creating Custom Variables and Factors" => "examples/basic_definingfactors.md",
            "Intermediate Tutorial: Creating DynPose Factor" => "examples/interm_dynpose.md",
            "Intermediate Tutorial: Fixed-Lag Solving" => "examples/interm_fixedlag_hexagonal.md"
        ],
        "Function Reference" => "func_ref.md"
    ]
    # html_prettyurls = !("local" in ARGS),
    )


deploydocs(
    repo   = "github.com/JuliaRobotics/Caesar.jl.git",
    target = "build",
    deps   = nothing,
    make   = nothing,
    julia  = "0.7",
    osname = "linux"
)
