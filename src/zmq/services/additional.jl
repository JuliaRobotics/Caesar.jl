
export
  multiplyDistributions,
  status

function status(cfg, fg, requestDict)
  @info "got status request"
  Dict{String, Any}("status" => "OK")
end

function multiplyDistributions(cfg, fg, requestDict)::Dict{String, Any}
  requestVal = Unmarshal.unmarshal(MultiplyDistributionsRequest, requestDict["payload"])

  funcs = BallTreeDensity[]

  for pbd in requestVal
    pts = pbd.points
    @warn "recalculating new bandwidth for multiplyDistributions"
    # TODO assuming 1D Euclidean
    p = AMP.manikde!( pts, (:Euclid,))
    push!(funcs, p)
  end

  pqr = manifoldProduct(funcs, (:Euclid,))


  bw = getBW(pqr)[1]
  n = Npts(pqr)
  rpts = getPoints(pqr)
  resObj = Packed_BallTreeDensity(1, "Gaussian", bw, 1/n*ones(n), rpts)

  Dict{String, Any}("status" => "OK", "payload" => resObj)
end
