

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

function buildtagdict(q::Rotations.Quat,
                      tvec::Union{Vector{Float64},StaticArrays.SArray{Tuple{3},<:Real,1,3}},
                      tagsize::Float64 )
  #
  onetag = Dict{Symbol, Any}()
  onetag[:pos] = tvec[:]
  onetag[:quat] = Float64[q.w; q.x; q.y; q.z]
  onetag[:tagxy] = Float64[tagsize; tagsize]
  onetag[:bearing] = [atan2(-tvec[1], tvec[3]);]
  onetag[:range] = [norm(tvec[[1;3]]);]
  onetag
end
function buildtagdict(q::Rotations.Quat,
                      tvec::CoordinateTransformations.Translation{T},
                      tagsize::Float64 ) where T
  buildtagdict(q, tvec.v, tagsize)
end

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
  ret, rvec, tvec = cv2.solvePnP(objPts, imgPts, camK, distCoeffs)

  Rmat = zeros(3,3)
  rodrigues!(Rmat,rvec[:])
  q = convert(Quat, RotMatrix{3}(Rmat))
  return q, Translation(SVector(tvec...))
end
# objPts.push_back(cv::Point3f(-s,-s, 0));
# objPts.push_back(cv::Point3f( s,-s, 0));
# objPts.push_back(cv::Point3f( s, s, 0));
# objPts.push_back(cv::Point3f(-s, s, 0));

# objectPoints = np.random[:random]((4,3,1))
# imagePoints = np.random[:random]((4,2,1))
# cameraMatrix = np.eye(3)
