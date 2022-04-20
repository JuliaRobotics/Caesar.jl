# user functions

@doc raw"""
    $SIGNATURES

Transform and put 2D pointcloud as [x,y] coordinates of a-frame into `aP_dest`, by transforming 
incoming b-frame points `bP_src` as [x,y] coords via the transform `aTb` describing the b-to-a (aka a-in-b)
relation.  Return the vector of points `aP_dest`.

````math
{}^a \begin{bmatrix} x, y \end{bmatrix} = {}^a_b \mathbf{T} \, {}^b \begin{bmatrix} x, y \end{bmatrix}
````
"""
function _transformPointCloud2D!(
    # manifold
    M::typeof(SpecialEuclidean(2)),
    # destination points
    aP_dest::AbstractVector,
    # source points
    bP_src::AbstractVector,
    # transform coordinates
    aCb::AbstractVector{<:Real}; 
    # base point on manifold about which to do the transform
    e0 = ProductRepr(SVector(0.0,0.0), SMatrix{2,2}(1.0,0.0,0.0,1.0)),
    backward::Bool=false
  )
  #
  
  aTb = retract(M, e0, hat(M, e0, aCb))
  aTb = backward ? inv(M,aTb) : aTb
  bT_src = ProductRepr(MVector(0.0,0.0), SMatrix{2,2}(1.0,0.0,0.0,1.0))
  aT_dest = ProductRepr(MVector(0.0,0.0), MMatrix{2,2}(1.0,0.0,0.0,1.0))
  
  for (i,bP) in enumerate(bP_src)
    bT_src.parts[1] .= bP
    Manifolds.compose!(M, aT_dest, aTb, bT_src)
    # aP_dest[i][:] = Manifolds.compose(M, aTb, bP_src[i]).parts[1]
    aP_dest[i] .= aT_dest.parts[1]
  end

  aP_dest
end

function _transformPointCloud2D(
    # manifold
    M::typeof(SpecialEuclidean(2)),
    # source points
    bP_src::AbstractVector,
    # transform coordinates
    aCb::AbstractVector{<:Real}; 
    kw...
  )
  #
  #dest points
  aP_dest = typeof(MVector(0.0,0.0))[MVector(0.0,0.0) for _ in 1:length(bP_src)] # Vector{typeof(MVector(0.0,0.0))}(undef, length(bP_src))
  # fill!(aP_dest, MVector(0.0,0.0))
  _transformPointCloud2D!(M, aP_dest, bP_src, aCb; kw...)
end




## ===============================================================
## LEGACY BELOW, TODO DEPRECATE OBSOLETE OR OUTDATED FUNCTIONS BELOW
## ===============================================================

"""
    identitypose6fg()

Initialize a and exiting or new factor graph with a first pose and prior as specified by default
keywords or user.
"""
function identitypose6fg!(;fg=nothing,
      N::Int=100,
      initCov::Array{Float64,2}=0.001*eye(6),
      initpose::SE3=SE3(0) )
  #
  fg = fg != nothing ? fg : initfg()
  N = 100

  println("Adding PriorPose3 and :x0 to graph...")
  # pts = 0.001*randn(6,N)
  initPosePrior = PriorPose3( MvNormal(veeEuler(initpose), initCov)  )
  pts = getSample(initPosePrior,N)[1]
  v0 = addVariable!(fg, :x0,  pts,  N=N, tags=[:POSE;])
  f1  = addFactor!(fg,[v0], initPosePrior)
  return fg
end

function identitypose6fg(;fg=nothing,
      N::Int=100,
      initCov::Array{Float64,2}=0.001*eye(6),
      initpose::SE3=SE3(0) )
   @warn "Caesar.identitypose6fg deprecated, use identitypose6fg! instead"
   identitypose6fg!(fg=fg, N=N, initCov=initCov, initpose=initpose)
end


function hasval(d::Dict, va)
  for (k,v) in d
    if v == va
      return true
    end
  end
  return false
end


function projectrbe(fgl::G, from::Symbol, to::Vector{Float64}) where G <: AbstractDFG
  x1 = getKDEMax(getBelief(fgl,from))
  x2 = to

  wTc = SE3(x1[1:3],Euler(x1[4:6]...))

  cX2 = (matrix(wTc) \ [x2[1:3];1.0])[1:3]

  # Dx = x2[1:3]-x1[1:3]
  Dx = cX2
  range = norm(Dx)
  bearing = atan2(Dx[2],Dx[1])
  elev = -atan2(Dx[3],norm(Dx[1:2]))
  return range, bearing, elev
end


function projectrbe(fgl::G, from::Symbol, to::Symbol) where G <: AbstractDFG
  # x1 = getKDEMax(getBelief(fg,from))
  x2 = getKDEMax(getBelief(fgl,to))
  return projectrbe(fgl, from, x2[1:3])

  # wTc = SE3(x1[1:3],Euler(x1[4:6]...))
  #
  # cX2 = (matrix(wTc) \ [x2;1.0])[1:3]
  #
  # # Dx = x2[1:3]-x1[1:3]
  # Dx = cX2
  # range = norm(Dx)
  # bearing = atan2(Dx[2],Dx[1])
  # elev = -atan2(Dx[3],norm(Dx[1:2]))
  # return range, bearing, elev
end





















#
