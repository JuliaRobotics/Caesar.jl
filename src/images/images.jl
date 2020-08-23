
@info "Loading Caesar tools related to Images.jl."

export writevideo

"""
    $SIGNATURES

Use ffmpeg to write image sequence to video file.

Notes:
- Requires Images.jl
- https://discourse.julialang.org/t/creating-a-video-from-a-stack-of-images/646/8
"""
function writevideo(fname::AbstractString, 
                    imgstack::Array{<:Color,3};
                    overwrite=true, fps::Int=30, options=``)
  #
  ow = overwrite ? `-y` : `-n`
  h, w, nframes = size(imgstack)
  open(`ffmpeg
      -loglevel warning
      $ow
      -f rawvideo
      -pix_fmt rgb24
      -s:v $(h)x$(w)
      -r $fps
      -i pipe:0
      $options
      -vf "transpose=0"
      -pix_fmt yuv420p
      $fname`, "w") do out
    for i = 1:nframes
      write(out, convert.(RGB{N0f8}, clamp01.(imgstack[:,:,i])))
    end
  end
end
