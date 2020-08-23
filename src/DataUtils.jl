
export getGitCommitSHA


"""
    $SIGNATURES

Return the git commit for the current package directory used for `pkgname`.

Notes
- Taken from: https://discourse.julialang.org/t/how-to-get-the-commit-of-head-of-a-julia-package/10877
"""
function getGitCommitSHA(package_name::AbstractString)
  gitdir = joinpath(Pkg.dir(package_name), ".git")
  commit = strip(readstring(`git --git-dir $gitdir rev-parse HEAD`))
  return commit
end




#
