# Utils for foveation

export
  foveateQueryToPoint



function foveateQueryToPoint{T <: AbstractString}(cg::CloudGraph,
        sessions::Vector{T};
        point::Vector{Float64}=[0.0;0.0],
        minrange::Number=1.5, maxrange::Number=5, fovrad::Number=0.3  )
  #
  neoids, syms = Vector{Int}(), Vector{Symbol}()
  loadtx = transaction(cg.neo4j.connection)
  nms = length(sessions)
  query = "MATCH (n) WHERE ("
  cnt = 0
  for session in sessions
    query *= "(n:$(session)) "
    cnt+=1
    if cnt < nms
      query *= " or "
    else
      query *= ") and "
    end
  end
  query *=
  "// Region filtering
  	($(point[1])-n.MAP_est[0])*($(point[1])-n.MAP_est[0]) +
  	($(point[2])-n.MAP_est[1])*($(point[2])-n.MAP_est[1]) > $(minrange^2)
  AND
  	($(point[1])-n.MAP_est[0])*($(point[1])-n.MAP_est[0]) +
  	($(point[2])-n.MAP_est[1])*($(point[2])-n.MAP_est[1]) < $(maxrange^2)
  AND
  // Frustum cutoff filtering
  	(abs(atan2( ($(point[2])-n.MAP_est[1]),
  	($(point[1])-n.MAP_est[0])) - n.MAP_est[2] ) < $(fovrad)
  OR
  	abs(atan2( ($(point[2])-n.MAP_est[1]),
  	($(point[1])-n.MAP_est[0])) - n.MAP_est[2] + 6.28) < $(fovrad)
  OR
  	abs(atan2( ($(point[2])-n.MAP_est[1]),
  	($(point[1])-n.MAP_est[0])) - n.MAP_est[2] - 6.28) < $(fovrad))
  RETURN id(n), n.label"
  cph = loadtx(query, submit=true)
  for res in cph.results[1]["data"]
    push!(neoids, res["row"][1]  )
    push!(syms, Symbol(res["row"][2])  )
  end
  return neoids, syms
end

function foveateQueryToPoint(cg::CloudGraph,
        session::AbstractString;
        point::Vector{Float64}=[0.0;0.0],
        minrange::Number=1.5, maxrange::Number=5, fovrad::Number=0.3  )
  #
  foveateQueryToPoint(cg::CloudGraph,
          [session],
          point=point,
          minrange=minrange, maxrange=maxrange, fovrad=fovrad  )
end
