using Documenter, Caesar

makedocs(
    modules = [Caesar],
    format = :html,
    sitename = "Caesar.jl",
    pages = Any[
        "Home" => "index.md",
        "Examples" => "examples.md",
        "Tutorial 1: ContinuousScalar" => "tutorialcontinuousscalar.md",
        "Tutorial 2: Camera + AprilTags Calibration" => "tutorialcamcal.md",
        "Arena Visualization" => "arena_visualizations.md",
        "Database Layer" => "database_interactions.md",
        "Functions" => "func_ref.md"
    ]
    # html_prettyurls = !("local" in ARGS),
    )


deploydocs(
    repo   = "github.com/dehann/Caesar.jl.git",
    target = "build",
    deps   = nothing,
    make   = nothing,
    julia  = "0.6",
    osname = "linux"
)
