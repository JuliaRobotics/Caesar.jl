

function getmongokeys(fgl::FactorGraph, x::Symbol, IDs)
  cvid = -1
  # TODO -- could likely just use existing mapping already in fgl
  for id in IDs
    if Symbol(fgl.g.vertices[id[1]].label) == x
      cvid = id[2]
      break
    end
  end
  if cvid == -1
    warn("getmongokeys is not finding $(x)")
    return Dict{AbstractString, Any}(), cvid
  end
  # @show cvid
  cv = CloudGraphs.get_vertex(fgl.cg, cvid)
  if haskey(cv.properties, "mongo_keys")
    jsonstr = cv.properties["mongo_keys"]
    return JSON.parse(jsonstr), cvid
  else
    return Dict{AbstractString, Any}(), cvid
  end
end

function fetchmongorgbimg(cg::CloudGraph, key::AbstractString)
  myKeyToFind = BSONOID(key)
  findsomthing = find(cg.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
  myFavouriteKey = first( findsomthing );
  data = myFavouriteKey["val"]
  img = ImageMagick.readblob(data);

  # r, c = size(img)
  # imgA = zeros(r,c,3);
  # for i in 1:r, j in 1:c
  #   imgA[i,j,1] = img[i,j].r
  #   imgA[i,j,2] = img[i,j].g
  #   imgA[i,j,3] = img[i,j].b
  # end
  return img
end

function bin2arr(data::Vector{UInt8}; dtype::DataType=Float64)
  len = length(data)
  dl = sizeof(dtype)
  alen = round(Int,len/dl)

  ptrT = pointer(data)  # Ptr{UInt8}
  ptrTf = convert(Ptr{dtype}, ptrT)  # Ptr{Float32}
  arr = Vector{dtype}(alen);

  unsafe_copy!(pointer(arr),ptrTf,alen)

  return arr
end

function fetchmongodepthimg(cg::CloudGraph, key::AbstractString; dtype::DataType=Float64)
  myKeyToFind = BSONOID(key) # some valid numbers

  findsomthing = find(cg.mongo.cgBindataCollection, ("_id" => eq(myKeyToFind)))
  myFavouriteKey = first( findsomthing );
  mfkv = myFavouriteKey["val"];

  return bin2arr(mfkv, dtype=dtype)
  # len = length(mfkv)
  # dl = sizeof(dtype)
  # alen = round(Int,len/dl)
  #
  # ptrT = pointer(mfkv)  # Ptr{UInt8}
  # ptrTf = convert(Ptr{dtype}, ptrT)  # Ptr{Float32}
  # arr = Vector{dtype}(alen);
  #
  # unsafe_copy!(pointer(arr),ptrTf,alen)
  #
  # return arr
end






function findAllBinaryFactors(cgl::CloudGraph, session::AbstractString)
  xx = ls(cgl, session)

  slowly = Dict{Symbol, Tuple{Symbol, Symbol, Symbol}}()
  @showprogress 1 "Finding all binary edges..." for (x,va) in xx
    facts = ls(cgl, session, sym=x)
    for (fc, va2) in facts
      nodesdict = ls(cgl, session, sym=fc)
      if length(nodesdict) == 2
        # add to dictionary for later drawing
        nodes = collect(keys(nodesdict))
        if !haskey(slowly, fc) && !haselement(nodesdict[nodes[1]][3],:FACTOR) && !haselement(nodesdict[nodes[2]][3],:FACTOR)
          # fv = getVert(fgl, fgl.fIDs[fc])
          # vty = typeof(getfnctype(fv)).name.name
          slowly[fc] = (nodes[1], nodes[2], Symbol("NEEDCACHING"))
        end
      end
    end
  end

  return slowly
end


function drawLineBetween!(vis::DrakeVisualizer.Visualizer,
        cgl::CloudGraph,
        session::AbstractString,
        fr::Symbol,
        to::Symbol;
        scale=0.01,
        name::Symbol=:edges,
        subname::Union{Void,Symbol}=nothing,
        color=RGBA(0,1.0,0,0.5)  )
  #

  cv1 = getCloudVert(cgl, session, sym=fr)
  v1 = cloudVertex2ExVertex(cv1)
  cv2 = getCloudVert(cgl, session, sym=to)
  v2 = cloudVertex2ExVertex(cv2)

  drawLineBetween!(vis,session,v1,v2,scale=scale,name=name,subname=subname,color=color   )
  nothing
end



function drawAllBinaryFactorEdges!(vis::DrakeVisualizer.Visualizer,
      cgl::CloudGraph,
      session::AbstractString;
      scale=0.01  )
  #

  sloth = findAllBinaryFactors(cgl, session)

  @showprogress 1 "Drawing all binary edges..." for (teeth, toe) in sloth
    color = pointToColor(toe[3])
    drawLineBetween!(vis, cgl, session, toe[1], toe[2], subname=toe[3], scale=scale, color=color)
  end
  nothing
end


#
