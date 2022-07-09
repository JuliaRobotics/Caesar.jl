# 



export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners
export generateCostAprilTagsPreimageCalib

# suppressing but listing some internal functions
# export _defaultCameraCalib, _AprilTagToPose2


"""
    $TYPEDEF

Simplified constructor type to convert between 4 corner detection of AprilTags to a Pose2Pose2 factor for use in 2D

Notes
- Coordinate frames are:
  - assume robotics body frame is xyz <==> fwd-lft-up
  - assume AprilTags pose is xyz <==> rht-dwn-fwd
  - assume camera frame is xyz <==> rht-dwn-fwd
  - assume Images.jl frame is row-col <==> i-j  <==> dwn-rht
- Helper constructor uses `f_width, f_height, c_width, c_height,s` to build `K`, 
  - setting `K` will overrule `f_width,f_height, c_width, c_height,s`.
- Finding preimage from deconv measurement sample `idx` in place of MvNormal mean:
  - see [`generateCostAprilTagsPreimageCalib`](@ref) for detauls.

Example

```julia
# bring in the packages
using AprilTags, Caesar, FileIO

# the size of the tag, as in the outer length of each side on of black square 
taglength = 0.15

# load the image
img = load("photo.jpg")

# the image size
width, height = size(img)
# auto-guess `f_width=height, c_width=round(Int,width/2), c_height=round(Int, height/2)`

detector = AprilTagDetector()
tags = detector(img)

# new factor graph with Pose2 `:x0` and a Prior.
fg = generateGraph_ZeroPose(varType=Pose2)

# use a construction helper to add factors to all the tags
for tag in tags
  tagSym = Symbol("tag\$(tag.id)")
  exists(fg, tagSym) ? nothing : addVariable!(fg, tagSym, Pose2)
  pat = Pose2AprilTag4Corners(corners=tag.p, homography=tag.H, taglength=taglength)
  addFactor!(fg, [:x0; tagSym], pat)
end

# free AprilTags library memory
freeDetector!(detector)
```

DevNotes
- TODO IIF will get plumbing to combine many of preimage `obj` terms into single calibration search

Related

`AprilTags.detect`, `PackedPose2AprilTag4Corners`, [`generateCostAprilTagsPreimageCalib`](@ref)
"""
struct Pose2AprilTag4Corners{T <: SamplableBelief, F <: Function} <: IIF.AbstractManifoldMinimize
  # 4 corners as detected by AprilTags
  corners::NTuple{4,Tuple{Float64,Float64}}
  # homography matrix
  homography::Matrix{Float64}
  # camera calibration
  K::Matrix{Float64}
  # define the size of the tag in meters
  taglength::Float64
  # tag id
  id::Int
  # internally computed relative factor between binary variables-- from camera to tag in camera or body frame
  Z::RoME.Pose2Pose2{T}

  # experimental for 'SLAM-aware' camera intrinsic calibration -- stores (lambda function, starting estimate), see [`generateCostAprilTagsPreimageCalib`](@ref).
  preimage::Tuple{F, Vector{Float64}}
end

getManifold(::IIF.InstanceType{<:Pose2AprilTag4Corners}) = getManifold(Pose2Pose2) # SE2E2_Manifold


Base.propertynames(::Pose2AprilTag4Corners, private::Bool=false) = private ? (:Zij, :Z) : (:Z,)

Base.getproperty(x::Pose2AprilTag4Corners,f::Symbol) = begin
    if f == :Zij
      @warn "Pose2AprilTag4Corners field .Zij is deprecated, use .Z instead."
      getfield(x, :Z)
    else
      getfield(x,f)
    end
  end

"""
    $SIGNATURES

A default camera calibration matrix, useful for reference and quick tests.

Notes
- Hartley, Zisserman, Multiple View Geometry, p.157

DevNotes
- TODO likely to move to a common camera models Pkg
- TODO consolidate with similar functions in RoME?
"""
function _defaultCameraCalib(;f_width::Real = 480.0,
                              f_height::Real = 480.0,
                              c_width::Real = 320.0,
                              c_height::Real = 240.0,
                              s::Real  = 0.0)
  #
  K = [f_width s  c_width;
        0 f_height c_height;
        0 0  1.0]
  #
  return K
end


"""
    $SIGNATURES

Standardizing a function to convert a regular AprilTags.jl sighting into Pose2 format

Notes
- assume robotics body frame is xyz <==> fwd-lft-up
- assume AprilTags pose is xyz <==> rht-dwn-fwd
- assume camera frame is xyz <==> rht-dwn-fwd
- assume Images.jl frame is row-col <==> i-j  <==> dwn-rht
"""
function _AprilTagToPose2(corners, 
                          homography::AbstractMatrix{<:Real}, 
                          f_width::Real, 
                          f_height::Real, 
                          c_width::Real, 
                          c_height::Real, 
                          taglength_::Real)
  #
  # pose <==> cTt
  pose, err1 = AprilTags.tagOrthogonalIteration(corners, homography, f_width, f_height, c_width, c_height, taglength=taglength_)
  cVt = Translation((pose[1:3,4])...)
  # bRc = bRy * yRc 
  bRc = _Rot.QuatRotation(1/sqrt(2),0,1/sqrt(2),0) * _Rot.QuatRotation(1/sqrt(2),0,0,-1/sqrt(2))
  # for tag in body frame == bTt
  bTt = LinearMap(bRc) ∘ cVt
  
  # also extract the relative yaw between body and tag 
  ld = LinearAlgebra.cross(bTt.linear*pose[1:3,1:3]*[0;0;1], [0;0;1])
  theta = TU.wrapRad(atan(ld[2],ld[1]) + pi/2)
  
  M = getManifold(Pose2Pose2)
  e0 = getPointIdentity(M)

  # create a tangent vector of the measurement
  hat( M, e0, [bTt.translation[1:2,];theta] )
end

const _CornerVecTuple = Union{NTuple{4,Tuple{Float64,Float64}}, <:AbstractVector{Tuple{Float64,Float64}}, <:AbstractVector{<:AbstractVector{<:Real}}}

function Pose2AprilTag4Corners(;corners::_CornerVecTuple=((0.0,0.0),(1.0,0.0),(0.0,1.0),(1.0,1.0)),
                                homography::AbstractMatrix{<:Real}=diagm(ones(3)),
                                f_width::Real=480.0,
                                f_height::Real=f_width,
                                c_width::Real=320.0,
                                c_height::Real=240.0,
                                s::Real =0.0,
                                K::AbstractMatrix{<:Real}=_defaultCameraCalib(f_width=f_width,
                                                                              f_height=f_height,
                                                                              c_width=c_width,
                                                                              c_height=c_height,
                                                                              s=s),
                                id::Int=-1,
                                taglength::Real=0.25,
                                covariance::Matrix{<:Real}=diagm([0.1;0.1;0.1].^2) )
  #
  # make standard tuple
  corners_ = if corners isa Tuple
    corners
  else
    c = corners
    ((c[1][1],c[1][2]),(c[2][1],c[2][2]),(c[3][1],c[3][2]),(c[4][1],c[4][2]))
  end

  # calculate the transform
  # f_width, f_height, c_width, c_height = K[1,1], K[2,2], K[1,3], K[2,3]
  x0 = [K[1,1], K[2,2], K[1,3], K[2,3], taglength]
  Dtag = _AprilTagToPose2(corners, homography, x0...)
  # println("$(tag.id), $(ld[3]), $(round.(Dtag, digits=3))")
  M = SpecialEuclidean(2)
  e0 = getPointIdentity(M)
  p2l2 = Pose2Pose2(MvNormal(vee(M,e0,Dtag), covariance))

  # preimage optimize function
  ## TODO not the greatest use of Manifolds.jl, can do better
  preImgFnc = (y, x) -> sum( vee(M, e0, y - _AprilTagToPose2(corners_, homography, x...)).^2 )
  # preImgFnc = (y, x) -> sum( (y - _AprilTagToPose2(corners_, homography, x...)).^2 )

  #
  return Pose2AprilTag4Corners(corners_, homography, K, taglength, id, p2l2, (preImgFnc, x0))
end


function Pose2AprilTag4Corners( tag::AprilTag;
                                corners::_CornerVecTuple=tag.p,
                                homography::AbstractMatrix{<:Real}=tag.H,
                                f_width::Real=480.0,
                                f_height::Real=f_width,
                                c_width::Real=320.0,
                                c_height::Real=240.0,
                                s::Real =0.0,
                                K::AbstractMatrix{<:Real}=_defaultCameraCalib(f_width=f_width,
                                                                              f_height=f_height,
                                                                              c_height=c_height,
                                                                              c_width=c_width,
                                                                              s=s),
                                id::Int=tag.id,
                                taglength::Real=0.25,
                                covariance::Matrix{<:Real}=diagm([0.1;0.1;0.1].^2) )
  #
  # this is a lazy construction helper for direct use in Julia

  Pose2AprilTag4Corners(;corners=corners,
                        homography=homography,
                        f_width=f_width,
                        f_height=f_height,
                        c_width=c_width,
                        c_height=c_height,
                        s=s,
                        K=K,
                        id=id,
                        taglength=taglength,
                        covariance=covariance )
end


# just pass through sample and factor evaluations

function getSample( pat4c::CalcFactor{<:Pose2AprilTag4Corners} )
  #
  M = getManifold(pat4c.factor.Z)
  ϵ = getPointIdentity(Pose2)

  X = sampleTangent(M, pat4c.factor.Z.Z, ϵ)
  return X
end

function (pat4c::CalcFactor{<:Pose2AprilTag4Corners})(X,
                                                      p,
                                                      q)
  #

  # @assert X isa ProductRepr "Pose2AprilTag4Corners expects measurement sample X to be a Manifolds tangent vector, not coordinate or point representation.  Got X=$X"
  M = getManifold(pat4c.factor.Z)
  q̂ = Manifolds.compose(M, p, exp(M, getPointIdentity(M), X)) #for groups
  #TODO allocalte for vee! see Manifolds #412, fix for AD
  Xc = zeros(3)
  vee!(M, Xc, q, log(M, q, q̂))
  return Xc
  # wXjhat = SE2(wxi)*SE2(z)
  # jXjhat = SE2(wxj) \ wXjhat
  # se2vee(jXjhat)
end

## serialization

Base.@kwdef struct PackedPose2AprilTag4Corners <: AbstractPackedFactor
  # format of serialized data
  _type::String = "Caesar.PackedPose2AprilTag4Corners"
  # corners, as detected by AprilTags library
  corners::Vector{Float64}
  # homography matrix
  homography::Vector{Float64}
  # camera calibration
  K::Vector{Float64}
  # define the size of the tag in meters
  taglength::Float64
  # tag id
  id::Int
end

Base.propertynames(::PackedPose2AprilTag4Corners, private::Bool=false) = private ? (:mimeType, :_type) : (:_type,)

Base.getproperty(x::PackedPose2AprilTag4Corners,f::Symbol) = begin
    if f == :mimeType
      @warn "PackedPose2AprilTag4Corners field .mimeType is deprecated, use ._type instead."
      getfield(x, :_type)
    else
      getfield(x,f)
    end
  end

# not needed for immutable
# Base.setproperty!(x::PackedPose2AprilTag4Corners,f::Symbol,val) = begin
#   if f == :mimeType
#     @warn "PackedPose2AprilTag4Corners field .mimeType is deprecated, use ._type instead."
#     setfield!(x, :_type, val)
#   else
#     setfield!(x,f,val)
#   end
# end

function Base.convert(::Type{<:AbstractPackedFactor}, 
                      obj::Pose2AprilTag4Corners)
  #
  corVec = zeros(8)
  corVec[1] = obj.corners[1][1]
  corVec[2] = obj.corners[1][2]
  corVec[3] = obj.corners[2][1]
  corVec[4] = obj.corners[2][2]
  corVec[5] = obj.corners[3][1]
  corVec[6] = obj.corners[3][2]
  corVec[7] = obj.corners[4][1]
  corVec[8] = obj.corners[4][2]

  return PackedPose2AprilTag4Corners( "/application/JuliaLang/PackedPose2AprilTag4Corners",
                                      corVec, 
                                      obj.homography[:],
                                      obj.K[:],
                                      obj.taglength,
                                      obj.id )
end


function convert( ::Type{<:DFG.AbstractRelative}, 
                  obj::PackedPose2AprilTag4Corners)
  #
  cv = obj.corners
  corners = ((cv[1],cv[2]),(cv[3],cv[4]),(cv[5],cv[6]),(cv[7],cv[8]))
  
  return Pose2AprilTag4Corners( corners=corners, 
                                homography=reshape(obj.homography,:,3), 
                                K=reshape(obj.K,3,3),
                                taglength=obj.taglength,
                                id=obj.id  )
end




## calibrate via preimage

"""
    $SIGNATURES

Helper function to generate and calculate the aggregate cost indicative of the discrepancy between 
the deconvolution prediction and prescribed measurement of mutliple `Pose2AprilTag4Corners` factors
in the factor graph.

The idea is that a bad calibration will give some kind of SLAM result, and if there is enough information
then the SLAM result can be used to bootstrap better and better calibration estimates of the camera that
was used to capture the AprilTag sightings.  This function is meant to help do that secondary parameter 
search inside a factor graph objection, after a regular solution has been found.

Notes
- `pred, _ = approxDeconv(dfg, fct)`
- `fct.preimage[1](pred[:,idx], [f_width, f_height, c_width, c_height, taglength])`
  - `fct.preimage[1]` is a function to find the preimage.
- `obj = (fc_wh) -> fct.preimage[1](pred[:,idx], fc_wh)`
  - `fc_wh = [f_width, f_height, c_width, c_height, 0.172]`
- `obj2 = (fcwh) -> obj([fcwh[1]; fcwh[1]; fcwh[2]; c_height; taglength])`
- `result = Optim.optimize(obj, fct.preimage[2], BFGS(), Optim.options(x_tol=1e-8))`
  - A stored starting estimate for optimization `fct.preimage[2]`
"""
function generateCostAprilTagsPreimageCalib(dfg::AbstractDFG,
                                            fsyms::Vector{Symbol}=lsf(dfg, Pose2AprilTag4Corners );
                                            idx::Int = 1, # the sample number
                                            f_width::Real=480.0,
                                            f_height::Real=f_width,
                                            c_width::Real=320.0,
                                            c_height::Real=240.0,
                                            taglength::Real=0.172,
                                            args::Function = (f_cw)->[f_cw[1]; f_cw[1]; f_cw[2]; c_height; taglength]  )
  #

  # temporary memory containers for all the different lambda functions
  fcts = []
  objs = []
  obj2s = []
  preds = []
  
  for i in 1:length(fsyms)
    fsym = fsyms[i]
    pred, _ = approxDeconv(dfg, fsym)
    push!(preds, pred)
    fct = getFactorType(dfg, fsym)
    push!(fcts, fct)
    # fct.preimage[1](pred[:,idx], f_cw)
    obj = (fc_wh) -> fcts[i].preimage[1](preds[i][:,idx], fc_wh)
    push!(objs, deepcopy(obj))
    # use keyword args mapping on which parameter should be optimized
    obj2 = (f_cw) -> objs[i](args(f_cw))
    push!(obj2s, deepcopy(obj2))
  end
  
  # set up the cummulative cost over all functions in `obj2s` 
  cost = x->((f->f(x)).(obj2s) |> sum)

  # test the function is working
  # @show cost([f_width; c_width])

  return cost
end


##





#