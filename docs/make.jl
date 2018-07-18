using Documenter, Caesar

makedocs(
    modules = [Caesar],
    format = :html,
    sitename = "Caesar.jl",
    pages = Any[
        "Home" => "index.md",
        "Getting Started" => "getting_started.md",
        "Examples" => "examples.md",
        "Entry Tutorial: ContinuousScalar" => "tutorialcontinuousscalar.md",
        "Entry Tutorial: Hexagonal 2D SLAM" => "tut_hexagonal2d.md",
        "Entry Tutorial: Singular Ranges-only SLAM" => "tut_slamedonut.md",
        "Moderate Tutorial: Defining Factors" => "definingfactors.md",
        "Arena Visualization" => "arena_visualizations.md",
        "Offloading to Server" => "database_interactions.md",
        "Functions" => "func_ref.md"
    ]
    # html_prettyurls = !("local" in ARGS),
    )


deploydocs(
    repo   = "github.com/JuliaRobotics/Caesar.jl.git",
    target = "build",
    deps   = nothing,
    make   = nothing,
    julia  = "0.6",
    osname = "linux"
)
