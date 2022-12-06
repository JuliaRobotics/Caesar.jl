# From https://github.com/pglira/simpleICP/blob/02df330a63e81918ce82b95d849a585b775b0c85/julia/simpleicp.jl#L1
# 
# MIT License

# Copyright (c) 2020-2022 Philipp Glira
# Copyright (c) 2022-     Dehann Fourie

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


function _ICP_PointCloud(pc::PointCloud)
  len = length(pc)
  _ICP_PointCloud(
    pc,
    [zeros(3) for _ in 1:len],
    zeros(len),
    collect(1:len)
  )
end

Base.length(pc::_ICP_PointCloud) = length(pc.xyz)

function Base.getproperty(pc::_ICP_PointCloud, f::Symbol)
  if f == :no_points
    return length(pc)
  elseif f == :x
    @warn "Inefficient data access to `pc::_ICP_PointCloud.x`, access `pc.xyz.points[:][1]` instead." maxlog=10
    pc_ = pc.xyz.points
    return [pt.x for pt in pc_]
  elseif f == :y
    @warn "Inefficient data access to `pc::_ICP_PointCloud.y`, access `pc.xyz.points[:][3]` instead." maxlog=10
    pc_ = pc.xyz.points
    return [pt.y for pt in pc_]
  elseif f == :z
    @warn "Inefficient data access to `pc::_ICP_PointCloud.z`, access `pc.xyz.points[:][3]` instead." maxlog=10
    pc_ = pc.xyz.points
    return [pt.z for pt in pc_]
  elseif f == :nx
    @warn "Inefficient data access to `pc::_ICP_PointCloud.nx`, access `pc.nv[:][1]` instead." maxlog=10
    return [pt[1] for pt in pc.nv]
  elseif f == :ny
    @warn "Inefficient data access to `pc::_ICP_PointCloud.ny`, access `pc.nv[:][2]` instead." maxlog=10
    return [pt[2] for pt in pc.nv]
  elseif f == :nz
    @warn "Inefficient data access to `pc::_ICP_PointCloud.nz`, access `pc.nv[:][3]` instead." maxlog=10
    return [pt[3] for pt in pc.nv]
  end
  return getfield(pc, f)
end

function select_in_range!(pc::_ICP_PointCloud, X::Array, max_range::Number)
  size(X)[2] == 3 || error(""""X" must have 3 columns""")
  max_range > 0 || error(""""max_range" must be > 0""")
  kdtree = KDTree(X')
  query_points = [pc.x[pc.sel]'; pc.y[pc.sel]'; pc.z[pc.sel]']
  _, distances = nn(kdtree, query_points)
  keep = [d <= max_range for d in distances]
  pc.sel = pc.sel[keep]
end

function select_n_points!(pc::_ICP_PointCloud, n)
  no_selected_points = length(pc.sel)
  if no_selected_points > n
      idx = round.(Int, range(1, no_selected_points, length=n))
      pc.sel = pc.sel[idx]
  end
end

function estimate_normals!(pc::_ICP_PointCloud, neighbors)
  # pc = pc_.xyz.points
  kdtree = KDTree([pc.x'; pc.y'; pc.z'])
  query_points = [pc.x[pc.sel]'; pc.y[pc.sel]'; pc.z[pc.sel]']
  idxNN_all_qp, = knn(kdtree, query_points, neighbors, false)

  # @show typeof(idxNN_all_qp) size(idxNN_all_qp) idxNN_all_qp[1:5]

  for (i, idxNN) in enumerate(idxNN_all_qp)
    xyz_ = pc.xyz.points[idxNN]
    x_ = (s->s.data[1]).(xyz_)
    y_ = (s->s.data[2]).(xyz_)
    z_ = (s->s.data[3]).(xyz_)
    selected_points = [x_'; y_'; z_']
    # P = fit(PCA, selected_points, pratio=1.0)
    # pc.nx[pc.sel[i]] = projection(P)[1,3]
    # pc.ny[pc.sel[i]] = projection(P)[2,3]
    # pc.nz[pc.sel[i]] = projection(P)[3,3]
    C = Statistics.cov(selected_points, dims=2)
    F = LinearAlgebra.eigen(C) # eigenvalues are in ascending order
    pc.nv[pc.sel[i]] = F.vectors[:,1]
    pc.planarity[pc.sel[i]] = (F.values[2]-F.values[1])/F.values[3];
  end

end


function matching!(
    pcmov::_ICP_PointCloud, 
    pcfix::_ICP_PointCloud
  )
  #
  kdtree = KDTree([pcmov.x'; pcmov.y'; pcmov.z'])
  query_points = [pcfix.x[pcfix.sel]'
                  pcfix.y[pcfix.sel]'
                  pcfix.z[pcfix.sel]']
  idxNN, = knn(kdtree, query_points, 1)
  pcmov.sel = vcat(idxNN...)

  dx = pcmov.x[pcmov.sel] - pcfix.x[pcfix.sel]
  dy = pcmov.y[pcmov.sel] - pcfix.y[pcfix.sel]
  dz = pcmov.z[pcmov.sel] - pcfix.z[pcfix.sel]

  nx = pcfix.nx[pcfix.sel]
  ny = pcfix.ny[pcfix.sel]
  nz = pcfix.nz[pcfix.sel]

  distances = [dx[i]*nx[i] + dy[i]*ny[i] + dz[i]*nz[i] for i in 1:length(pcmov.sel)]

  return distances
end


function reject!(pcmov::_ICP_PointCloud, pcfix::_ICP_PointCloud, min_planarity, distances)

  planarity = pcfix.planarity[pcfix.sel]

  med = StatsBase.median(distances)
  sigmad = StatsBase.mad(distances, normalize=true)

  keep_distance = [abs(d-med) <= 3*sigmad for d in distances]
  keep_planarity = [p > min_planarity for p in planarity]

  keep = keep_distance .& keep_planarity

  pcmov.sel = pcmov.sel[keep]
  pcfix.sel = pcfix.sel[keep]
  deleteat!(distances, .!keep)

  return nothing

end


function check_convergence_criteria(distances_new, distances_old, min_change)

  change(new, old) = abs((new-old)/old*100)

  change_of_mean = change(mean(distances_new), mean(distances_old))
  change_of_std = change(std(distances_new), std(distances_old))

  return change_of_mean < min_change && change_of_std < min_change ? true : false

end


## ================================================================
## 
## ================================================================


function estimate_rigid_body_transformation(x_fix, y_fix, z_fix, nx_fix, ny_fix, nz_fix,
                                            x_mov, y_mov, z_mov)

    A = hcat(-z_mov.*ny_fix + y_mov.*nz_fix,
              z_mov.*nx_fix - x_mov.*nz_fix,
             -y_mov.*nx_fix + x_mov.*ny_fix,
             nx_fix,
             ny_fix,
             nz_fix)

    l = nx_fix.*(x_fix-x_mov) + ny_fix.*(y_fix-y_mov) + nz_fix.*(z_fix-z_mov)

    x = A\l

    residuals = A*x-l

    R = euler_angles_to_linearized_rotation_matrix(x[1], x[2], x[3])

    t = x[4:6]

    H = affine_matrix(_SE3_MANI, ArrayPartition(t, R))
    # H = create_homogeneous_transformation_matrix(R, t)

    return H, residuals

end

"""
    $SIGNATURES

Align two point clouds using ICP (with normals).

Example:

```julia
using Downloads, DelimitedFiles
using Colors, Caesar

# get some test data (~50mb download)
lidar1_url = "https://github.com/JuliaRobotics/CaesarTestData.jl/raw/main/data/lidar/simpleICP/terrestrial_lidar1.xyz"
lidar2_url = "https://github.com/JuliaRobotics/CaesarTestData.jl/raw/main/data/lidar/simpleICP/terrestrial_lidar2.xyz"
io1 = PipeBuffer()
io2 = PipeBuffer()
Downloads.download(lidar1_url, io1)
Downloads.download(lidar2_url, io2)

X_fix = readdlm(io1)
X_mov = readdlm(io2)

H, HX_mov, stat = Caesar._PCL.alignICP_Simple(X_fix, X_mov; verbose=true)
```

Notes
- Mostly consolidated with `Caesar._PCL` types.
- Internally uses `Caesar._PCL._ICP_PointCloud` which was created to help facilite consolidation of code:
  - Modified from www.github.com/pglira/simpleICP (July 2022).
- See here for a brief example on [Visualizing Point Clouds](@ref viz_pointcloud_makie).

DevNotes
- TODO switch rigid transfrom to `Caesar._PCL.apply` along with performance considerations, instead of current `transform!`.

See also: [`PointCloud`](@ref)
"""
function alignICP_Simple(
    X_fix::AbstractMatrix, 
    X_mov::AbstractMatrix;
    correspondences::Integer=1000,
    neighbors::Integer=10,
    min_planarity::Number=0.3,
    max_overlap_distance::Number=Inf,
    min_change::Number=3,
    max_iterations::Integer=100,
    verbose::Bool=false,
    H = Matrix{Float64}(I,4,4),
  )
  #
  size(X_fix)[2] == 3 || error(""""X_fix" must have 3 columns""")
  size(X_mov)[2] == 3 || error(""""X_mov" must have 3 columns""")
  10 <= correspondences || error(""""correspondences" must be >= 10""")
  0 <= min_planarity < 1 || error(""""min_planarity" must be >= 0 and < 1""")
  2 <= neighbors || error(""""neighbors" must be >= 2""")
  0 < min_change || error(""""min_change" must be > 0""")
  0 < max_iterations || error(""""max_iterations" must be > 0""")

  dt = @elapsed begin
    verbose && @info "Create point cloud objects ..."
    pcfix = _ICP_PointCloud(PointCloud(X_fix[:,1], X_fix[:,2], X_fix[:,3]))
    pcmov = _ICP_PointCloud(PointCloud(X_mov[:,1], X_mov[:,2], X_mov[:,3]))

    if isfinite(max_overlap_distance)
        verbose && @info "Consider partial overlap of point clouds ..."
        select_in_range!(pcfix, X_mov, max_overlap_distance)
        if length(pcfix.sel) == 0
            error(@sprintf("Point clouds do not overlap within max_overlap_distance = %.5f! ",
            max_overlap_distance) * "Consider increasing the value of max_overlap_distance.")
        end
    end

    verbose && @info "Select points for correspondences in fixed point cloud ..."
    select_n_points!(pcfix, correspondences)
    sel_orig = pcfix.sel

    verbose && @info "Estimate normals of selected points ..."
    estimate_normals!(pcfix, neighbors)

    residual_distances = Any[]

    verbose && @info "Start iterations ..."
    count = 0
    for i in 1:max_iterations
      count += 1
      initial_distances = matching!(pcmov, pcfix)
      reject!(pcmov, pcfix, min_planarity, initial_distances)
      dH, residuals = estimate_rigid_body_transformation(
        pcfix.x[pcfix.sel], pcfix.y[pcfix.sel], pcfix.z[pcfix.sel],
        pcfix.nx[pcfix.sel], pcfix.ny[pcfix.sel], pcfix.nz[pcfix.sel],
        pcmov.x[pcmov.sel], pcmov.y[pcmov.sel], pcmov.z[pcmov.sel])

      push!(residual_distances, residuals)
      transform!(pcmov, dH)
      H .= dH*H
      pcfix.sel = sel_orig
      if i > 1
        if check_convergence_criteria(residual_distances[i], residual_distances[i-1],
                                    min_change)
          verbose && @info "Convergence criteria fulfilled -> stop iteration!"
          break
        end
      end

      if verbose 
        if i == 1
          @info @sprintf(" %9s | %15s | %15s | %15s", "Iteration", "correspondences",
                          "mean(residuals)", "std(residuals)")
          @info @sprintf(" %9s | %15d | %15.4f | %15.4f", "orig:0", length(initial_distances),
                          mean(initial_distances), std(initial_distances))
        end
        @info @sprintf(" %9d | %15d | %15.4f | %15.4f", i, length(residual_distances[i]),
                        mean(residual_distances[i]), std(residual_distances[i]))
      end
    end
  end

  if verbose
    @info "Estimated transformation matrix H:\n" *
    @sprintf("[%12.6f %12.6f %12.6f %12.6f]\n", H[1,1], H[1,2], H[1,3], H[1,4]) *
    @sprintf("[%12.6f %12.6f %12.6f %12.6f]\n", H[2,1], H[2,2], H[2,3], H[2,4]) *
    @sprintf("[%12.6f %12.6f %12.6f %12.6f]\n", H[3,1], H[3,2], H[3,3], H[3,4]) *
    @sprintf("[%12.6f %12.6f %12.6f %12.6f]\n", H[4,1], H[4,2], H[4,3], H[4,4])
    @info "Finished in " * @sprintf("%.3f", dt) * " seconds!"
  end

  X_mov_transformed = [pcmov.x pcmov.y pcmov.z]

  stat = (;residual_mean=mean(residual_distances[end]), correspondences=length(residual_distances[end]),residual_std=std(residual_distances[end]), iterations=count)
  return H, X_mov_transformed, stat
end


##