
export getDataPointCloud, getPointCloud, getPointCloud2D, getPointCloud3D


function getDataPointCloud(
  nfg::AbstractDFG,
  varlbl, 
  pattern::Union{Symbol,UUID,<:AbstractString, Regex};
  getDatakws...
)
  # get point cloud blob
  try
    de,dbl = getData(nfg, Symbol(varlbl), pattern; getDatakws...)
    if isnothing(dbl)
      @error "could find in variable $varlbl, blob $pattern"
      return nothing
    end
    # FIXME, change serialization to more standard pcd or laz formats
    return dbl |> IOBuffer |> Serialization.deserialize
  catch err
    if err isa KeyError
      @error err
      return nothing
    end
    throw(err)
  end
  # unpack specific blob format
end



function getPointCloud(
  nfg::AbstractDFG,
  label,
  pattern = r"PCLPointCloud2", 
  w...;
  kw...
)
  pc2_a = getDataPointCloud(nfg, label, pattern)
  if pc2_a isa Nothing
    return nothing
  end
  pc_a = _PCL.PointCloud(pc2_a)
  return _prepPointCloud(pc_a, w...; kw...)
end

getPointCloud2D(
  nfg::AbstractDFG,
  label, 
  regex=r"PCLPointCloud2", 
  dims=1:2;
  minrange=0.0,
  maxrange=999.
) = getPointCloud(
  nfg, 
  label, 
  regex, 
  dims;
  minrange,
  maxrange
)

getPointCloud3D(
  nfg::AbstractDFG,
  label, 
  regex=r"PCLPointCloud2", 
  dims=1:3;
  minrange=0.0,
  maxrange=999.
) = getPointCloud(
  nfg, 
  label, 
  regex, 
  dims;
  minrange,
  maxrange
)
