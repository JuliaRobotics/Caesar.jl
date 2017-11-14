using Documenter, Caesar

makedocs(
    modules = [Caesar],
    format = :html,
    sitename = "Caesar.jl",
    pages = Any[
        "Home" => "index.md",
        "Examples" => "examples.md",
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
