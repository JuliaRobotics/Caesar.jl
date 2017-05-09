# Utils for foveation and spacial querying of the database

export
  whosNear2D,
  whosNear3D,
  foveateQueryToPoint



  """
      whosNear2D(cg::CloudGraph, session::AbstractString; x,y,yaw,dist,angle )

  Find vertices near the point specified and return dictionary of symbol to Neo4j ID pairs.
  """
  function whosNear2D(cg::CloudGraph, session::AbstractString;
        x=nothing,
        y=nothing,
        yaw=nothing,
        dist::Float64=3.0,
        angle::Float64=pi/12.0  )
    #
    if x==nothing && y==nothing && yaw==nothing
      error("please give some info on where you want to run the spacial query, xyYaw.")
    end
    # Build the query
    query = "match (n:$(session)) where exists(n.MAP_est) "
    query = x==nothing ? query : query*"and abs(n.MAP_est[0] - $(x)) < $(dist) "
    query = y==nothing ? query : query*"and abs(n.MAP_est[1] - $(y)) < $(dist) "
    query = yaw==nothing ? query : query*"and abs(n.MAP_est[2] - $(yaw)) < $(angle) "
    query = query*"return n.label, id(n)"

    cph, = executeQuery(cg, query)

    symneo = Dict{Symbol, Int}()
    for data in cph.results[1]["data"]
      symneo[Symbol(data["row"][1])] = data["row"][2]
    end
    # neoid = cph.results[1]["data"][1]["row"][1]

    symneo
  end

  """
      whosNear3D(cg::CloudGraph, session::AbstractString; x,y,z,roll, pitch,yaw,dist,angle )

  Find vertices near the point specified and return dictionary of symbol to Neo4j ID pairs.
  """
  function whosNear3D(cg::CloudGraph, session::AbstractString;
        x=nothing,
        y=nothing,
        z=nothing,
        roll=nothing,
        pitch=nothing,
        yaw=nothing,
        dist::Float64=3.0,
        angle::Float64=pi/12.0  )
    #
    if x==nothing && y==nothing && z==nothing && roll==nothing && pitch==nothing && yaw==nothing
      error("please give some info on where you want to run the spacial query, xyzRPY.")
    end
    # Build the query
    query = "match (n:$(session)) where exists(n.MAP_est) "
    query = x==nothing ? query : query*"and abs(n.MAP_est[0] - $(x)) < $(dist) "
    query = y==nothing ? query : query*"and abs(n.MAP_est[1] - $(y)) < $(dist) "
    query = z==nothing ? query : query*"and abs(n.MAP_est[2] - $(z)) < $(dist) "
    query = roll==nothing ? query : query*"and abs(n.MAP_est[3] - $(roll)) < $(angle) "
    query = pitch==nothing ? query : query*"and abs(n.MAP_est[4] - $(pitch)) < $(angle) "
    query = yaw==nothing ? query : query*"and abs(n.MAP_est[5] - $(yaw)) < $(angle) "
    query = query*"return n.label, id(n)"

    cph, = executeQuery(cg, query)

    symneo = Dict{Symbol, Int}()
    for data in cph.results[1]["data"]
      symneo[Symbol(data["row"][1])] = data["row"][2]
    end

    symneo
  end



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
