

function multiplyFunctions(cfg, fg, requestDict)::Dict{String, Any}
  requestVal = Unmarshal.unmarshal(MultiplyDistributionsRequest, requestDict["payload"])
  # @show


  funcs = BallTreeDensity[]

  for i in 1:??
    pts = 
    # TODO assuming 1D Euclidean
    p = AMP.manikde!( pts, (:Euclid,))
    push!(funcs, )
  end


  Dict{String, Any}()
end
