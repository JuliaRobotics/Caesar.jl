

function saveSlam(slamwrapper::SLAMWrapper; filename::AbstractString="tempSlam.jld")
  saveslam = deepcopy(slamwrapper);
  @warn "Saving factor graph and landmark index -- unresolved issue with saving current tree, but can be reccomputed from factor graph with IncrementalInference.wipeBuildNewTree!(...)."
  saveslam.tree = Union{}
  jldopen(filename,"w") do file
    write(file, "slam", saveslam)
  end
  nothing
end

function loadSlam(; filename::AbstractString="tempSlam.jld")
  d = jldopen(filename,"r") do file
    read(file, "slam", slam)
  end
  return d
end

function haselement(arr::Vector{T}, val::T) where {T}
  for a in arr
    if a == val
      return true
    end
  end
  return false
end
