
@info "Loading Caesar tools related to Images.jl."

# using Images
# using ImageTransformations

export writevideo
export imhcatPretty, csmAnimationJoinImgs, csmAnimateSideBySide

"""
    $SIGNATURES

Use ffmpeg to write image sequence to video file.

Notes:
- Requires Images.jl
- https://discourse.julialang.org/t/creating-a-video-from-a-stack-of-images/646/8
"""
function writevideo(fname::AbstractString, 
                    imgstack::AbstractArray{<:Colorant,3};
                    overwrite=true, fps::Int=30, options=``, 
                    player::AbstractString="",
                    pix_fmt="yuv420p" )
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
      -pix_fmt $pix_fmt
      $fname`, "w") do out
    for i = 1:nframes
      write(out, convert.(RGB{N0f8}, clamp01.(imgstack[:,:,i])))
    end
  end
  if 0 < length(player)
    @async run(`$player $fname`)
  end
end

function writevideo(fname::AbstractString,
                    imgs::AbstractVector{<:AbstractArray{<:Colorant,2}};
                    overwrite=true, fps::Int=30, options=``, 
                    player::AbstractString="",
                    pix_fmt="yuv420p" )
  #
  @cast imgstack[r,c,k] := imgs[k][r,c]
  writevideo(fname, imgstack, overwrite=overwrite, fps=fps, options=options, player=player, pix_fmt=pix_fmt)
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

function csmAnimationJoinImgs(folderpath::AbstractString = "/tmp/caesar/csmCompound/";
                              leftname::AbstractString="csm_",
                              rightname::AbstractString="tree_",
                              bothname::AbstractString="both_",
                              ext::AbstractString="png",
                              files::AbstractVector{<:AbstractString} = readdir(folderpath),
                              nrLt::Int = filter(x->occursin(ext,x),filter(x->occursin(leftname,x), files)) |> length,
                              nrRt::Int = filter(x->occursin(ext,x),filter(x->occursin(rightname,x), files)) |> length  )
  #
  # loop over all image pairs
  allFrames = 1:minimum([nrRt,nrLt])
  @showprogress "Side by side $(allFrames[end]) images" for idx in allFrames
    iml = load(joinpath(folderpath,"$(leftname)$idx.$ext"))
    imr = load(joinpath(folderpath,"$(rightname)$idx.$ext"))
    imb = imhcatPretty(iml, imr)
    # save new side by side image
    save(joinpath(folderpath,"$(bothname)$idx.$ext"), imb)
  end
  joinpath(folderpath,"$(bothname)$(allFrames[end]).$ext")
end


"""
    $SIGNATURES

Extension of `IIF.csmAnimate` that draws the `solveTree!(...; recordhists=..)` 
Bayes tree development alongside the CSM animation.

Example
-------

```julia

fg = generateCanonicalFG_Hexagonal()
tree, smt, hists = solveTree!(fg, recordcliqs=ls(fg))

# now generate all the video frames at default `folderpath=/tmp/caesar/csmCompound/`
csmAnimateSideBySide(tree, hists)

# and render the video using ffmpeg
run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/both_%d.png -c:v libtheora -vf fps=5 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -q 10 /tmp/caesar/csmCompound/out.ogv`)
@async run(`totem /tmp/caesar/csmCompound/out.ogv`)
```

DevNotes
- Likely possible to use `writevideo` or something similar.
- `folderpath` not fully populated everywhere so likely not working properly yet (help requested pls)
- `tree` not strictly needed, since `autohists` already has the tree structure stored inside it.

Related

IIF.csmAnimate, Caesar.writevideo
"""
function csmAnimateSideBySide(tree::BayesTree,
                              autohists::Dict{Int, T};
                              frames::Int=100,
                              interval::Int=2,
                              fps::Int=5,
                              rmfirst::Bool=true, 
                              fsmColors::Dict{Symbol,String}=Dict{Symbol,String}(),
                              defaultColor::AbstractString="lightpink",
                              folderpath::AbstractString="/tmp/caesar/csmCompound/",
                              show::Bool=false  ) where T <: AbstractVector
  #
  csmAnimate(tree, autohists, interval=interval, frames=frames, rmfirst=rmfirst, folderpath=folderpath, fsmColors=fsmColors, defaultColor=defaultColor )
  csmAnimationJoinImgs(folderpath)

  @info "reruns of `csmAnimate` will by default clear $folderpath and thereby remove any previous work done in that folder (including previously generated videos)."
  if show
    run(`ffmpeg -r 10 -i /tmp/caesar/csmCompound/both_%d.png -c:v libtheora -vf fps=$fps -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -q 10 /tmp/caesar/csmCompound/out.ogv`)
    @async run(`totem /tmp/caesar/csmCompound/out.ogv`)
  end
end


#