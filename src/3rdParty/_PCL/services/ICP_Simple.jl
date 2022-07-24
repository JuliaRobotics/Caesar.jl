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
    len,
    zeros(len),
    collect(1:len)
  )
end


function select_in_range!(pc::_ICP_PointCloud, X::Array, max_range::Number)

  size(X)[2] == 3 || error(""""X" must have 3 columns""")
  max_range > 0 || error(""""max_range" must be > 0""")

  kdtree = KDTree(X')

  query_points = [pc.xyz.points.x[pc.sel]'; pc.xyz.points.y[pc.sel]'; pc.xyz.points.z[pc.sel]']

  _, distances = nn(kdtree, query_points)

  keep = [d <= max_range for d in distances]

  pc.sel = pc.sel[keep]

end