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
    @warn "Inefficient data access to `pc::_ICP_PointCloud.nz`, access `pc.nv[:,3]` instead." maxlog=10
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
