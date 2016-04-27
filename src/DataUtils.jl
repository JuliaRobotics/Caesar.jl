

function saveSlam(slamwrapper::SLAMWrapper; filename::AbstractString="tempSlam.jld")
  saveslam = deepcopy(slamwrapper);
  warn("Saving factor graph and landmark index -- unresolved issue with saving current tree, but can be reccomputed from factor graph with IncrementalInference.wipeBuildNewTree!(...).")
  saveslam.tree = Union{}
  jldopen(filename,"w") do file
    write(file, "slam", slam)
  end
  nothing
end

function loadSlam(; filename::AbstractString="tempSlam.jld")
  slam = jldopen(filename,"r") do file
    read(file, "slam", slam)
  end
  return slam
end
