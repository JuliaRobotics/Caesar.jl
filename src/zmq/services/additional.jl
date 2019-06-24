
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

  for pbd in requestVal.weights
    pts = pbd.points
    @warn "recalculating new bandwidth for multiplyDistributions"
    # TODO assuming 1D Euclidean
    p = AMP.manikde!( reshape(pts, 1, :), (:Euclid,))
    push!(funcs, p)
  end

  pqr = manifoldProduct(funcs, (:Euclid,))

  bw = vec(getBW(pqr))
  n = Npts(pqr)
  rpts = vec(getPoints(pqr))
  resObj = Packed_BallTreeDensity(1, "Gaussian", bw, 1/n*ones(n), rpts)

  Dict{String, Any}("status" => "OK", "payload" => resObj)
end

# t2 = "{\"payload\":{\"weights\":[{\"distType\":\"BallTreeDensity\",\"kernelType\":\"Gaussian\",\"weights\":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0],\"bandwidth\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],\"points\":[-0.8272759752535751,1.90340735325013,0.5617238720356695,-1.2610130840544629,-0.5196580171571705,0.47808234574398156,-0.5034354386657888,-0.1289061606999511,-0.6283440136328498,-0.04520468574277743],\"dim\":1},{\"distType\":\"BallTreeDensity\",\"kernelType\":\"Gaussian\",\"weights\":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0],\"bandwidth\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],\"points\":[0.6986754683541115,0.4206784705438769,-0.01798641761511103,0.2862258908507545,0.5021835960347083,-1.4424531814460273,0.7667313775874391,0.13048774810077268,-0.004778323705883176,-0.6408606329686699],\"dim\":1}]},\"request\":\"multiplyDistributions\"}"
