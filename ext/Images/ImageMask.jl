
function makeMaskImage(
  img::AbstractMatrix, 
  nodes::AbstractVector = [
    (180.0, 0.0);  # ll
    (10.0, 600.0); # lr
    (0.0, 600.0);  # ur
    (0.0, 0.0);    # ul
  ],
)
  h,w = size(img)
  # h,w = 400,640
  buffer = zeros(UInt8, h, w)
  
  points = []
  for nd in nodes
    push!(points, GeoPr.Point(nd...))
  end
  # (ll,lr,ur,ul)
  poly = GeoPr.Polygon(points...)
  
  # populate buffer
  for x in collect(1:1:h), y in collect(1:1:w)
    if GeoPr.inpolygon(poly, GeoPr.Point(x, y)) 
      buffer[x, y] = UInt8(1) 
    end 
  end

  return buffer, poly
end


"""
    $SIGNATURES


```julia
jstr_Mask_BOT = \"""
[
  [[180.0,0.0],[10.0,600.0],[0.0,600.0],[0.0,0.0]],
  [[400.0,0.0],[400.0,640.0],[310.0,640.0],[315.0,165.0],[360.0,0.0],[399.0,0.0]]
]
\"""

mnodes = JSON3.read(jstr_Mask_BOT)
mask_u8, polys = makeMaskImages(img, mnodes)
```
"""
function makeMaskImages(
  img::AbstractMatrix,
  mnodes::AbstractVector{<:AbstractVector}
)
  mask = zeros(UInt8, size(img)...)
  polys = []

  for nodes in mnodes
    @show nodes
    mk, poly = makeMaskImage(img, nodes)
    mask += mk
    push!(polys, poly)
  end

  return mask, polys
end


"""
    $SIGNATURES

Apply color to masked region onto a return copy of the image.
"""
function applyMaskImage(
  img::AbstractMatrix{<:Colorant},
  mask::AbstractMatrix,
  color::Colorant = RGB{N0f8}(0.0,0.4,0.4)
)
  imask = mask.*-1 .+ 1
  suppr = RGB.(mask)  .⊙ img
  keepr = RGB.(imask) .⊙ img
  color .⊙ suppr + keepr 
end



function imhcatPretty(iml::AbstractMatrix{<:Colorant},
                      imr::AbstractMatrix{<:Colorant} )
  #
  imll = similar(iml)
  fill!(imll, RGB{N0f8}(1,1,1))

  # where to place imr
  heightratio, widthratio = size(imll,1)/size(imr,1), size(imll,2)/size(imr,2)   
  minratio = 0.9*minimum([heightratio, widthratio])
  imrr = Images.imresize(imr, ratio=minratio)
  offsr = round.(Int, 0.05*[size(imll)...])
  endir = [size(imrr)...] + offsr
  imll[offsr[1]:endir[1]-1,offsr[2]:endir[2]-1] .= imrr

  hcat(iml,imll)
end
