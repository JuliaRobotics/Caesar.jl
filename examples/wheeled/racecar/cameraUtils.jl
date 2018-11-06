

function rodrigues!(Rmat::Array{Float64,2}, rvec::Vector{Float64})
  th = norm(rvec)
  rotv = rvec./th
  cth = cos(th)
  rotv1c = rotv.*(1-cth)

  Rmat[1,1] = 0.0
  Rmat[1,2] = -rotv[3]
  Rmat[1,3] = rotv[2]
  Rmat[2,1] = rotv[3]
  Rmat[2,2] = 0
  Rmat[2,3] = -rotv[1]
  Rmat[3,1] = -rotv[2]
  Rmat[3,2] = rotv[1]
  Rmat[3,3] = 0
  Rmat .*= sin(th)

  Rmat[1,1] += rotv1c[1]*rotv[1]
  Rmat[1,2] += rotv1c[1]*rotv[2]
  Rmat[1,3] += rotv1c[1]*rotv[3]
  Rmat[2,1] += rotv1c[2]*rotv[1]
  Rmat[2,2] += rotv1c[2]*rotv[2]
  Rmat[2,3] += rotv1c[2]*rotv[3]
  Rmat[3,1] += rotv1c[3]*rotv[1]
  Rmat[3,2] += rotv1c[3]*rotv[2]
  Rmat[3,3] += rotv1c[3]*rotv[3]

  Rmat[1,1] += cth
  Rmat[2,2] += cth
  Rmat[3,3] += cth
  nothing
end

function buildtagdict(cTt,
                      Ql::Rotations.Quat,
                      tvec, # ::Union{Vector{Float64},StaticArrays.SArray{Tuple{3},<:Real,1,3}}
                      tagsize::Float64,
                      bTcl )
  #
  # @show tvec, q
  # @show cTt = tvec ∘ LinearMap(Ql)
  @show bTcl, cTt
  @show bTt = bTcl ∘ cTt
  bP2t = getTagPP2(bTt)

  onetag = Dict{Symbol, Any}()
  onetag[:pos] = tvec.translation[:]
  onetag[:quat] = Float64[Ql.w; Ql.x; Ql.y; Ql.z]
  onetag[:tagxy] = Float64[tagsize; tagsize]
  onetag[:bearing] = [atan(-tvec.translation[1], tvec.translation[3]);]
  onetag[:range] = [norm(tvec.translation[[1;3]]);]
  # onetag[:tRYc] = convert(RotXYZ, q).theta2
  onetag[:bP2t] = bP2t
  onetag
end
# function buildtagdict(cTt,
#                       q::Rotations.Quat,
#                       tvec::CoordinateTransformations.Translation{T},
#                       tagsize::Float64,
#                       bTcl ) where T
#   buildtagdict(cTt, q, tvec, tagsize, bTcl )
# end

function getAprilTagTransform(tag::AprilTag,
                              camK::Array{Float64,2},
                              k1::Float64,
                              k2::Float64,
                              tagsize::Float64 )
  #
  imgPts = zeros(4,2,1)
  for i in 1:4, j in 1:2
    imgPts[i,j,1] = tag.p[i][j]
  end

  tsz = tagsize/2.0
  objPts = zeros(4,3,1)
  objPts[1,1,1] = -tsz; objPts[1,2,1] = -tsz;
  objPts[2,1,1] =  tsz; objPts[2,2,1] = -tsz;
  objPts[3,1,1] =  tsz; objPts[3,2,1] =  tsz;
  objPts[4,1,1] = -tsz; objPts[4,2,1] =  tsz;

  distCoeffs = zeros(5)
  distCoeffs[1] = k1
  distCoeffs[2] = k2

  # Python OpenCV
  # ret, rvec, tvec = cv2.solvePnP(objPts, imgPts, camK, distCoeffs)
  # Rmat = zeros(3,3)
  # rodrigues!(Rmat,rvec[:])
  # q = convert(Quat, RotMatrix{3}(Rmat))

  cTt = AprilTags.homographytopose(tag.H, camK[1,1], camK[2,2], camK[1,3], camK[2,3], taglength=tagsize)

  q = Quat(cTt[1:3,1:3])
  return q, Translation(SVector(cTt[1:3,4]...)), camK
end
# objPts.push_back(cv::Point3f(-s,-s, 0));
# objPts.push_back(cv::Point3f( s,-s, 0));
# objPts.push_back(cv::Point3f( s, s, 0));
# objPts.push_back(cv::Point3f(-s, s, 0));

# objectPoints = np.random[:random]((4,3,1))
# imagePoints = np.random[:random]((4,2,1))
# cameraMatrix = np.eye(3)

function getAprilTagTransform(tag::AprilTag;
                              fx::Float64=100.0,
                              fy::Float64=100.0,
                              cx::Float64=320.0,
                              cy::Float64=240.0,
                              k1::Float64=0.0,
                              k2::Float64=0.0,
                              tagsize::Float64=0.172 )
  camK = [fx 0 cx; 0 fy cy; 0 0 1]
  getAprilTagTransform(tag,
                       camK,
                       k1,
                       k2,
                       tagsize )
end


function prepTagBag(TAGS)
  tvec = Translation(0.0,0,0)
  q = Quat(1.0,0,0,0)
  # package tag detections for all keyframes in a tag_bag
  tag_bag = Dict{Int, Any}()
  for psid in 0:(length(TAGS)-1)
    tag_det = Dict{Int, Any}()
    for tag in TAGS[psid+1]
      q, tvec = getAprilTagTransform(tag, camK, k1, k2, tagsize)
      cTt = tvec ∘ CoordinateTransformations.LinearMap(q)
      tag_det[tag.id] = buildtagdict(cTt, q, tvec, tagsize, bTc)
    end
    tag_bag[psid] = tag_det
  end
  return tag_bag
end

# get Pose2Pose2 tag orientation transform
function getTagPP2(bTt)
  @show bTt
  cVz = LinearMap(Quat(bTt.linear))([1.0;0;0])
  wYt = atan(cVz[2],cVz[1])
  Translation(bTt.translation[1],bTt.translation[2],0) ∘ LinearMap(RotZ(wYt))
end


#
