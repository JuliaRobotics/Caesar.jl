#

# Task
# move linear at constant velocity along x axis, 10m/s, starting from 0

# Track the state separate like a filter by using underlying Factor Graph operations instead.
# the next example will keep the data in the same factor, and achieve objective more efficiently.

using IncrementalInference

# state
# ContinuousScalar

# Prediction
# LinearConditional

# measurement
# Prior



function forwardConvolve(X0::Array{Float64,2}, model)

  fg = initfg()

  addVariable!(fg, :x0, ContinuousScalar)
  manualinit!(fg, :x0, X0)
  addVariable!(fg, :x1, ContinuousScalar)

  addFactor!(fg, [:x0;:x1], model)

  ## TODO -- dont use name here, add API to just use z2 here
  return approxConv(fg, :x0x1f1, :x1)
end


## Start

# first numerical values -- samples from the marginal of X0
z1 = Normal(0,0.1)
X0 = rand(z1, 1,100)


## predict -- project / conv
# 0 -> 1 seconds
# make approx function
z2 = Normal(11,1.0) # odo
statemodel = LinearConditional( z2 )
X1_ = forwardConvolve(X0, statemodel)



## measure -- product of beliefs, using `ApproxManifoldProducts.jl`

predX1 = manikde!(X1_, ContinuousScalar)
z3 = Normal(9.5,0.75)
measX1 = manikde!(reshape(rand(z3,100),1,:), ContinuousScalar)

# do actual product
posterioriX1 = predX1 * measX1
X1 = getPoints(posterioriX1)


## predict, 1->2 seconds
z4 = Normal(8,2.0) # odo
statemodel = LinearConditional( z4 )
X2_ = forwardConvolve(X1, statemodel)


# measure (product)
z5 = Normal(20.8,2.0)
measX2 = manikde!(reshape(rand(z5,100),1,:), ContinuousScalar)
posterioriX2 = predX2 * measX2
X2 = getPoints(posterioriX2)


# predict
# ...


# measure


# predict


# measure










#
