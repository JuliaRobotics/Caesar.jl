# Resolve 





# 1. This is the (constant) scalar field the elevation measurements refer to
#
# struct ScalarField1D
#     # scalar field over one variable f:R->R (i.e. h= f(x))
#     # [a special case of ScalarField, h = f(·)]
#     # ...
# end

# 2. This is the actual scalar measurement sequence factor
# It takes two arguments: 
#  - the elevation measurement sequence, and 
#  - the odometry estimate at each elevation measurement
#
# addFactor!(fg, [:x1, :topography], ScalarFieldSequenceFactorNorthSouth(x,y))
struct ScalarFieldSequenceFactorNorthSouth{T} <: IIF.AbstractRelativeMinimize
  # FIXME: objective is to <: IIF.AbstractRelativeConstant (final name TBD)
  prevSequence::Vector{Float64} # sample location (odometry)
  nextSequence::Vector{Float64} # sample value (elevation meas)
  partial::T
end


# 3. Objective/fitness function evaluating match between a map and a template for different displacement values 
# This is used within the factor to generate the likelihood over the terrain
#
# NOTE: this function assumes map and template are on an equivalent grid 
# this function does not care for where/what the grid is
# NOTE: result will assume base pose is first in sequence
# for each i
#   energy[i] = sum( (map[i:(i+length(template))] - template).^2 )
# density = exp.(-energy)     # Woodward
# returns vector the size of length(map)
function _mySSDCorr(map, template)

    maplen = length(map)
    len = length(template)
    
    mnm = Statistics.mean(map)
    map_ = [map; mnm*ones(len-1)]
    density = zeros(maplen)
    for i in 1:maplen      
      density[i] = -sum( (map_[i:(i+len-1)] .- template).^2 )
    end
    adaptive = abs(maximum(density))

    density ./= adaptive
    density .= exp.(density)
    density ./= sum(density)
    return density
end


function IIF.getSample(s::CalcFactor{<:ScalarFieldSequenceFactorNorthSouth}, N::Int=1)
  # locality if desired from current variable estimate, 
  # X0 = getBelief(s.fullvariables[1]) # to consider only local map info

  # assuming user addConstant!(fg, :terrain, ScalarField1D(dem))

  # global terrain # should come from s.constants[:terrain] ?????

  # ensure equal spacing as terrain
  # assumptions:
  # - terrain.t is equally spaced
  # - s.factor.gridSequence increases monotonically
  # s.factor.gridSequence is the sequence of odometry estimates at which
  #  the elevation measurements (s.factor.scalarSequence) were taken
    # measurement = LinearInterpolation(s.factor.scalarSequence,s.factor.gridSequence)

    # upsample = 1 # avoid quantization effects from gridded sequence/terrain data
    # dx = diff(terrain.t)[1]/upsample
    # template = measurement.(s.factor.gridSequence[1]:dx:s.factor.gridSequence[end])
    # map_ = [terrain.u;]
    # map2_grid = range(terrain.t[1],terrain.t[end],length=upsample*length(map_))
    # map2 = terrain.(map2_grid)

  intensity_ = _mySSDCorr(prevSequence, nextSequence )
  
  
  # AliasingSS only works for 1D at this time
  # NOTE, if you stray out of region of support, things blow up!
  bss = AliasingScalarSampler([map2_grid;], intensity_) # [terrain.t;]
      
  # buffer with kde for continuous sampling (not grid spaced)
  stagedsmpls = rand(bss,N)
  smpls = rand(kde!(stagedsmpls, [3*dx;]),N)

  return (reshape(smpls,1,N),)
end




