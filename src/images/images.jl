
@info "Loading Caesar tools related to Images.jl."

# using Images
# using ImageTransformations

using ColorVectorSpace
using .Images

export applyMaskImage
export imhcatPretty

"""
    $SIGNATURES

Also see: [`toROSImage`](@ref)
"""
function toImage(msgd::Dict{String,Any})
  data = haskey(msgd, "data_b64") ? base64decode(msgd["data_b64"]) : UInt8.(msgd["data"])
  h, w = msgd["height"], msgd["width"]
  @show h, w, msgd["step"]
  if msgd["encoding"] == "mono8" || msgd["encoding"] == "8UC1"
    # img_ = normedview(N0f8, data)
    # reshape(img_, w, h)'
    img = Matrix{Gray{N0f8}}(undef, h, w)
    # assuming one endian type for now, TODO both little and big endian
    for i in 1:h, j in 1:w
      img[i,j] = Gray{N0f8}(data[msgd["step"]*(i-1)+j]/(2^8 - 1))
    end
    img
  elseif msgd["encoding"] == "mono16" || msgd["encoding"] == "16UC1"
    img_ = normedview(N0f16, data)
    reshape(img_, w, h)'
  else
    error("Conversion for ROS sensor_msgs.Image encoding not implemented yet $(msgd["encoding"])")
  end
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



#