# plot factor graph using Makie for belief space representation

using Distributed

using Caesar, RoME, DistributedFactorGraphs
@everywhere using Caesar, RoME, DistributedFactorGraphs
@everywhere using KernelDensityEstimate
@everywhere using ProgressMeter

@everywhere import RoME: Point2, Point3
using Makie
using MakieLayout
using DocStringExtensions

import DistributedFactorGraphs: getEstimates

"""
    $SIGNATURES

Get the cartesian range over which the factor graph variables span.

Notes:
- Optional `regexFilter` can be used to subselect, according to label, which variable IDs to use.

DevNotes
- TODO, allow `tags` as filter too.
"""
function getRangeCartesian(dfg::AbstractDFG,
                           regexFilter::Union{Nothing, Regex}=nothing;
                           extend::Float64=0.2,
                           digits::Int=6,
                           xmin::Real=99999999,
                           xmax::Real=-99999999,
                           ymin::Real=99999999,
                           ymax::Real=-99999999  )
  #
  # which variables to consider
  vsyms = getVariableIds(dfg, regexFilter)

  # find the cartesian range over all the vsyms variables

  for vsym in vsyms
    lran = getKDERange(getVariable(dfg, vsym) |> getKDE)
    xmin = lran[1,1] < xmin ? lran[1,1] : xmin
    ymin = lran[2,1] < ymin ? lran[2,1] : ymin
    xmax = xmax < lran[1,2] ? lran[1,2] : xmax
    ymax = ymax < lran[2,2] ? lran[2,2] : ymax
  end

  # extend the range for looser bounds on plot
  xra = xmax-xmin; xra *= extend
  yra = ymax-ymin; yra *= extend
  xmin -= extend; xmax += extend
  ymin -= extend; ymax += extend

  # clamp to nearest integers
  xmin = floor(xmin, digits=digits); xmax = ceil(xmax, digits=digits)
  ymin = floor(ymin, digits=digits); ymax = ceil(ymax, digits=digits)

  return [xmin xmax; ymin ymax]
end


"""
    $SIGNATURES

2D plot of variable marginal belief estimates.

Notes:
- Uses `Makie.contour` as backend
- Disable with `fadeFloor=0`, must be ∈ [0,1].
- `autoStride::Int` if at default `varStride==-1`, else ignore `autoStride`

DevNotes
- TODO, allow `tags` as filter too.

Example
-------
```julia
fg = generateCanonicalFG_Hexagonal()
pl, Z = plotVariableBeliefs(fg, r"x\\d") # using optional Regex filter
```

Related

getRangeCartesian,
"""
function plotVariableBeliefs(dfg::AbstractDFG,
                             regexFilter::Union{Nothing, Regex}=nothing;
                             vsyms::Vector{Symbol}=getVariableIds(dfg, regexFilter),
                             extras::Vector{Symbol}=Symbol[],
                             N::Int=100,
                             minColorBase::Float64=-0.3,
                             maxColorBase::Float64=1.0,
                             sortVars::Bool=false,
                             varStride::Int=-1,
                             autoStride::Int=300,
                             fade::Int=0,
                             fadeExtras::Bool=false,
                             fadeFloor::Real=0.3,
                             # fadeClamp::Bool=true,
                             tail::Int=-1,
                             digits::Int=-1,
                             extend::Real=0.2,
                             resolution::Union{Nothing, Tuple{Int, Int}}=nothing,
                             xmin::Real=99999999,
                             xmax::Real=-99999999,
                             ymin::Real=99999999,
                             ymax::Real=-99999999,
                             scale::Float64=1.0,
                             origin=(0,0),
                             scene=resolution==nothing ? Scene() : Scene(resolution=resolution)  )
  #
  # get range over which to plot
  dfgran = getRangeCartesian(dfg, regexFilter, digits=digits, extend=extend,
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

  x = LinRange(dfgran[1,1], dfgran[1,2], N)
  y = LinRange(dfgran[2,1], dfgran[2,2], N)
  Z = zeros(N,N)
  zz = zeros(N,N)
  xy = zeros(Threads.nthreads(), 2,N)
  for i in 1:Threads.nthreads()
    xy[i,2,:] .= y
  end

  # vsyms are variables for plotting, while applying available filters

  # specialty feature
  sortVars ? (vsyms .= vsyms |> sortDFG) : nothing
  # add additional variables that dont fit regex (AFTER SORTING)
  fadeExtras ? reverse!(vsyms) : nothing
  # add extras to front or back depending on fading request
  union!(vsyms, extras)
  fadeExtras ? reverse!(vsyms) : nothing
  varStride = varStride != -1 ? varStride : Int(floor(length(vsyms)/autoStride))+1
  sortVars && varStride != 1 ? (vsyms = vsyms[1:varStride:end]) : nothing
  sortVars && 0 < tail ? (vsyms = vsyms[end-tail:end]) : nothing
  !sortVars && varStride != 1 ? @warn("set sortVars=true to use varStride") : nothing
  !sortVars && 0 < fade ? @warn("set sortVars=true to use fade > 0") : nothing
  # walk through all variables and plotting accordingly
  len = length(vsyms)
  count = 0

  @showprogress "Evaluating symbols" for vsym in vsyms[1:end]
    if !isInitialized(getVariable(dfg, vsym))
      @warn "skipping belief plot of $vsym since not initialized"
      continue
    end
    count += 1
    XY = marginal(getVariable(dfg, vsym) |> getKDE, [1;2])
    Threads.@threads for i in 1:N
      xy[Threads.threadid(), 1,:] .= x[i]
      zz[i,:] = XY(xy[Threads.threadid(), :,:])
    end
    # normalize all beliefs to same scope
    zz ./= maximum(zz)
    # do the requested fading
    fadeval = maximum( [(fade-(len-count))/fade; 0.0]) * (1-fadeFloor) + fadeFloor
    zz .*= fadeval

    if isnan(maximum(zz)) || isnan(minimum(zz))
      @warn "skipping $vsym"
      continue
    end

    # Accumulate and clamp max value of accumulated beliefs
    Z .+= zz
    if count < len-fade && 0 < fadeFloor
      # i.e. count still in fade tail, must now do fade clamping
      Z[Z .> fadeFloor] .= fadeFloor
    else
      Z[Z .> 1] .= 1
    end
    # fade == -1 ? (Z .*= count/len) : nothing # alternatively
  end

  # set the base "background" color level by dropping one element to the desired minimum
  Z[1,end] += minColorBase
  Z[1,end-1] = maxColorBase

  # finally use Makie to draw the figure
  Makie.contour!(scene, scale.*x.+origin[1], scale.*y.+origin[2], Z, levels = 0, linewidth = 0, fillrange = true), Z
end






function addLinesBelief!(fg, pl, TTm;
                         scale::Float64=1.0,
                         origin=(0,0))
  ## This is a little excessive, but doesnt really matter
  # set all main variables and factors solvable
  setSolvable!(fg, :l1, 1)
  drtf = lsf(fg, r"drt")
  drts = ls(fg, r"drt")
  # set all necessary variables solvable
  map(v->setSolvable!(fg, v, 1), ls(fg, r"x\d"))
  vsyms = ls(fg, r"x\d", solvable=1) |> sortDFG
  # set all necessary factors solvable
  fcSo = union(map(v->setdiff(ls(fg, v), drtf), vsyms)...)
  map(v->setSolvable!(fg, v, 1), fcSo)
  # disallow factors connected to not solvables
  filter!(x->isSolved(fg, x), vsyms)
  vsyNo = setdiff(ls(fg, r"x\d",solvable=0), vsyms)
  fcNo = union(map(v->ls(fg, v), vsyNo)...)
  map(v->setSolvable!(fg, v, 0), fcNo)

  # time mask the syms
  asyms = ls(fg, r"x\d") |> sortDFG
  atims = getTimestamp.(map(x->getVariable(fg, x),asyms))
  tmask = atims .< getTimestamp(getVariable(fg, asyms[end]))

  # now draw vsyms
  # pl = plotVariableBeliefs(fg, r"x\d", vsyms=vsyms, sortVars=true, fade=15, fadeFloor=0.2)

  # overlay lines
  XYTv = map(x->getVariablePPE(getVariable(fg, x)).suggested, vsyms)
  XYT = hcat((v->v[1]).(XYTv), (v->v[2]).(XYTv), (v->v[3]).(XYTv))

  drttm = TTm .< getTimestamp(getVariable(fg, asyms[end]))
  XXmm, YYmm = XXm[drttm], YYm[drttm]
  # draw slam PPE suggested solution
  try
    lines!(pl, scale.*XYT[:,1].+origin[1], scale.*XYT[:,2].+origin[2], color=:black)
    lines!(pl, scale.*XXmm.+origin[1], scale.*YYmm.+origin[2], color=:red)
  catch ex
    @error ex
  end
  pl
end

0

# fg = generateCanonicalFG_Hexagonal()
# pl = plotVariableBeliefs(fg, r"x\d") # using optional Regex filter
# pl = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=3) # using optional Regex filter
# pl = plotVariableBeliefs(fg, r"x\d", sortVars=true, fade=2, tail=4) # using optional Regex filter





# targetResultsDir = "2020-01-08T17:57:25.612"
# targetResultsDir = "2020-01-15T18:30:40.98"
# fg = LightDFG{SolverParams}(params=SolverParams())
# loadDFG("/tmp/caesar/$targetResultsDir/fg_final.tar.gz", Main, fg)
# getSolverParams(fg).logpath = "/tmp/caesar/$targetResultsDir"
# dontMarginalizeVariablesAll!(fg)

# scene = plotVariableBeliefs(fg, r"x\d", sortVars=true, varStride=2, fade=20, digits=-2, fadeFloor=0.1)

# mask = YYf .< -32
# drt_data = readdlm(joinLogPath(fg, "DRT.csv"), ',')
# ... from GenerateResults.jl
# XXfm = XXf[mask]; YYfm = YYf[mask];
# lines!(scene, XXfm, YYfm, color=:red)

# xyt = getPPESuggestedAll(fg2, r"x\d")
# lines!(scene, xyt[2][:,1], xyt[2][:,2], color=:black)




# N = 20
# x = LinRange(-0.3, 1, N)
# y = LinRange(-1, 0.5, N)
# z = x .* y'
# hbox(
#    vbox(
#        contour(x, y, z, levels = 20, linewidth =3),
#        contour(x, y, z, levels = 0, linewidth = 0, fillrange = true),
#        heatmap(x, y, z),
#    ),
#    vbox(
#        image(x, y, z, colormap = :viridis),
#        surface(x, y, fill(0f0, N, N), color = z, shading = false),
#        image(-0.3..1, -1..0.5, AbstractPlotting.logo())
#    )
# )
