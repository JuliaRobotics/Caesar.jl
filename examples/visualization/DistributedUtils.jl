

function plotToVideo_Distributed( pltfnc::Function, 
                                  args::Union{<:AbstractVector, <:Tuple}; 
                                  pool = WorkerPool(procs()[2:end]),
                                  videofile::AbstractString="/tmp/test.avi" )
  # prep locations for image results
  len = length(args)
  _PL = Vector{Any}(undef, len)
  #
  pool = WorkerPool(procs()[2:end])
  for (i,lb) in enumerate(args)
    # Threads.@spawn begin
      im = pltfnc(lb)
      _PL[i] = remotecall(convert, pool, Matrix{RGB}, im)
      @show i
      nothing
    # end
  end
  # fetch the remote results
  PL = fetch.(_PL)

  Caesar.writevideo(videofile, PL, fps=5)

  PL, videofile
end