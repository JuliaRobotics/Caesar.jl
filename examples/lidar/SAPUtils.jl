

function alignPointCloudsSAP_2D(
  dfg, 
  vla, 
  vlb;
  usethreading = false,
  N = 10,              # number of alignments to draw statistics, this is for robustness or non-Gaussian
  sample_count = 1000, # computational load, bigger value is more accurate
  κ = 1e-3,            # this parameter was previously set for Radar, this value is better for LaserScan2D
  γ = 1e-1,            # best value TBD, this value dictates sensitivity and region of attraction
)
  @info "aligning pair $vla <-> $vlb"

  M = SpecialEuclidean(2)
  ϵ = ArrayPartition([0;0.],diagm([1;1.]))

  # get point cloud points
  source_pcm = getPointCloud2D(dfg, vla)
  dest_pcm   = getPointCloud2D(dfg, vlb)
  if isnothing(source_pcm) || isnothing(dest_pcm)
    return nothing
  end

  @cast pts_p[i][d] := source_pcm[i,d]
  @cast pts_q[i][d] := dest_pcm[i,d]
  @assert size(pts_p,2) == 2
  # pts_p = (s->s[1:2]).(pts_p_)
  # pts_q = (s->s[1:2]).(pts_q_)

  cloud1 = manikde!(Position2, pts_p, bw=κ*[1.0;1.0])
  cloud2 = manikde!(Position2, pts_q, bw=κ*[1.0;1.0])
  sap = ScatterAlignPose2(;cloud1, cloud2, bw=γ, sample_count)
  # dont push factor to DB, just sample locally using a temporary graph
  # could also be done in cloud
  tfg = initfg()
  # specific internal parameter probably not worth tweaking in this use case
  getSolverParams(tfg).inflateCycles = 1
  addVariable!(tfg, :x_a, Pose2)
  addVariable!(tfg, :x_b, Pose2)
  fsap = addFactor!(tfg, [:x_a; :x_b], sap; graphinit=false)
  # only a few samples
  sap_alg = Vector{Any}(undef,N)
  resids = Vector{Float64}(undef,N)
  tsks = Task[]
  threadwrapper(_i) = begin
    println("alignment ",_i)
    cf = CalcFactor(IIF._getCCW(fsap))
    sap_alg[_i] = sampleFactor(cf)[1]
    resids[_i]  = cf.cache.score[]
  end
  # Also recover the alignment residuals by using a for loop
  for i in 1:N
    # TODO, multiple threads are still slow and not confident yet on shared memory colisions
    if usethreading
      push!(tsks, Threads.@spawn threadwrapper($i))
    else
      threadwrapper(i)
    end
  end
  wait.(tsks)
  p_Ts_q = exp.(Ref(M), Ref(ϵ), sap_alg)
  b_Cpqs = vee.(Ref(M), Ref(ϵ), sap_alg)
  p_T_q = mean(M, p_Ts_q)
  b_Cpq_cov = cov(M, p_Ts_q)
  # p_T_q, fitness, inlier_rmse, correspondence_set = open3d_registration_2d_icp(source_pcm, dest_pcm)
  # 2D homography
  p_H_q = affine_matrix(M, p_T_q)

  b_Cpq = vee(M, ϵ, log(M, ϵ, p_T_q))
  Dict(
    "p_H_q" => p_H_q[:], 
    "p_Hsap_q" => p_H_q[:],
    "b_Cpq" => b_Cpq,
    "b_Cpq_cov" => b_Cpq_cov[:],
    "b_Cpqs" => b_Cpqs,
    "N" => N, 
    "sample_count" => sample_count,
    "kappa" => κ,
    "gamma" => γ,
    "residual_mean" => Statistics.mean(resids), 
    "residual_std" => Statistics.std(resids),
    "residuals" => resids,
    "algorithm" => "SAP_2D",
  )
end


function alignPoseToPose(
  dfg::AbstractDFG,
  vlp::Symbol,
  vlq::Symbol;
  cjlicp=true,
  th_distance = 3.0,
  pattern::Regex=r"PCLPointCloud2"
)
  M = SpecialEuclidean(3)
  p_PCp = _PCL.getDataPointCloud(dfg, vlp, pattern; checkhash=false) |> _PCL.PointCloud
  q_PCq = _PCL.getDataPointCloud(dfg, vlq, pattern; checkhash=false) |> _PCL.PointCloud

  w_T_p = getBelief(dfg, vlp, :parametric) |> mean
  w_T_q = getBelief(dfg, vlq, :parametric) |> mean
  phat_T_q = compose(M, inv(M, w_T_p), w_T_q)

  phat_PCq = _PCL.apply(M, phat_T_q, q_PCq)

  p_H_phat, p_PCq, status = if cjlicp
    pHphat, p_PCq_, status = _PCL.alignICP_Simple(p_PCp, phat_PCq)
    p_PCq = _PCL.PointCloud(p_PCq_)
    pHphat, p_PCq, status
  else
    p_pts = (p->SVector(p.x,p.y,p.z)).(p_PCp.points)
    q_pts = (p->SVector(p.x,p.y,p.z)).(phat_PCq.points)
    @cast p_ptsM[i,d] := p_pts[i][d] # fixed
    @cast q_ptsM[i,d] := q_pts[i][d] # gest transformed
    pHphat, residual_mean, inlier_rmse, correspondence_set = open3d_registration_3d_icp(
      p_ptsM, 
      q_ptsM;
      th_distance 
    )
    p_T_phat = ArrayPartition(SVector(pHphat[1:end-1,end]...), SMatrix{3,3}(pHphat[1:end-1,1:end-1]))
    p_PCq = _PCL.apply(SpecialEuclidean(3), p_T_phat, phat_PCq)
    pHphat, p_PCq, (;inlier_rmse, residual_mean)
  end

  p_H_q = p_H_phat * affine_matrix(M, phat_T_q)

  p_PCp, q_PCq, p_H_q, p_PCq
end