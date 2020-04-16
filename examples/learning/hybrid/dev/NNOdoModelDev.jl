## Python reference

ENV["PYTHON"] = "/usr/bin/python3.6"

using Pkg
Pkg.build("PyCall")

using PyCall

# py"""
# import sys
# sys.path.insert(0, "/home/singhk/learning-odometry/")
# """
py"""
import sys
sys.path.insert(0, "/home/dehann/.julia/dev/Caesar/examples/learning/hybrid/dev/")
"""

Base.cd("/home/dehann/.julia/dev/Caesar/examples/learning/hybrid/dev/")

# np = pyimport("numpy")
getM = pyimport("model_weights0")
pywe = getM.getWeights()


pymodels = pyimport("PyNNModels")

# pymodel = tf.keras.models.load_model("/tmp/caesar/conductor/sim_models/original_network0.h5")
pymodel = tf.keras.models.load_model("/tmp/caesar/conductor/models/retrained_network_model0.h5")
pywe = pymodel.get_weights()

pyX = [rand(4) for i in 1:25]

pymodel.predict([[pyX]])

## Build a full new model in this context to test end to end first

pmF = pymodels.build_model((25,4))
pmF.set_weights(pywe)
@assert pmF.predict([[pyX]]) - pymodel.predict([[pyX]]) .|> abs |> sum < 1e-10 "Newly constructed model does not match"



## in Julia

using Flux

# load data in near similar format
jlX = zeros(25,4)
for i in 1:25
  jlX[i,:] .= pyX[i]
end

# W1 = [
# -0.06407804 -0.07884349  0.00956099 -0.21626501 -0.01075477 0.21383737  0.07711418  0.16997789;
# -0.06179927 -0.08914955  0.08072356 -0.06508656 -0.01248774 0.16290735  0.03741108  0.12805876;
# -0.89473826  0.53830796 -0.25319445 -0.19648497 -2.4377484  0.916801   -0.46429503 -1.0824044;
# -3.5242891  -0.48591098  0.96419954  0.77881175  0.71329623 0.00865639 -3.323312    0.14445505
# ]' |> collect
#
# b1 = [0.07736943, -0.05445341, -0.03332321, -0.08276157, 0.06977537, 0.04422938, -0.04833093, -0.18779308]


## test the first Dense layer

W1 = pywe[1]
b1 = pywe[2]

c1_1 = (jlX*W1)' .+ b1

# Build first portion of model to compare with Flux

pm1_1 = pymodels.build_model1_1((25,4))
pm1_1.set_weights(pywe[1:2])
c1_1_py = pm1_1.predict([[pyX]])

for i in 1:8
  @assert c1_1_py[1,:,i] - c1_1[i,:] |> norm < 1e-4
end

## test with relu activation

c1 = (jlX*W1)' .+ b1 .|> relu


pm1 = pymodels.build_model1((25,4))
pm1.set_weights(pywe[1:2])
c1_py = pm1.predict([[pyX]])

for i in 1:8
  @assert c1_py[1,:,i] - c1[i,:] |> norm < 1e-4
end


## test with max pooling

c1_3 = reshape(c1', 25,8,1)
c2 = maxpool(c1_3, PoolDims(c1_3, 4))

modc = Chain(
  x -> (x*W1)' .+ b1 .|> relu,
  x -> reshape(x', 25,8,1),
  x -> maxpool(x, PoolDims(x, 4))
)

c2 = modc(jlX)


pm2 = pymodels.build_model2((25,4))
pm2.set_weights(pywe[1:2])
c2_py = pm2.predict([[pyX]])

for i in 1:6
  @assert c2_py[1,i,:] - c2[i,:,1] |> norm < 1e-4
end


## test flatten layer

c3__ = reshape(c2[:,:,1]',1,:)

modc = Chain(
  x -> (x*W1)' .+ b1 .|> relu,
  x -> reshape(x', 25,8,1),
  x -> maxpool(x, PoolDims(x, 4)),
  # x -> reshape(x[:,:,1]',1,:),
  x -> reshape(x[:,:,1]',:),
)

c3 = modc(jlX)

pm3 = pymodels.build_model3((25,4))
pm3.set_weights(pywe[1:2])
c3_py = pm3.predict([[pyX]])

@assert c3_py[:] - c3[:] |> norm < 1e-6


##  Test full model


modjl = Chain(
  x -> (x*W1)' .+ b1 .|> relu,
  x -> reshape(x', 25,8,1),
  x -> maxpool(x, PoolDims(x, 4)),
  # x -> reshape(x[:,:,1]',1,:),
  x -> reshape(x[:,:,1]',:),
  Dense(48,8,relu),
  Dense(8,2)
)

modjl[5].W .= pywe[3]'
modjl[5].b .= pywe[4]

modjl[6].W .= pywe[5]'
modjl[6].b .= pywe[6]

c5 = modjl(jlX)

c5_py = pymodel.predict([[pyX]])

@assert c5_py[:] - c5[:] |> norm < 1e-6



##  Utility functions to take values from tf


function buildPyNNModel_01_FromElements(W1::AbstractMatrix{<:Real}=zeros(4,8),
                                        b1::AbstractVector{<:Real}=zeros(8),
                                        W2::AbstractMatrix{<:Real}=zeros(8,48),
                                        b2::AbstractVector{<:Real}=zeros(8),
                                        W3::AbstractMatrix{<:Real}=zeros(2,8),
                                        b3::AbstractVector{<:Real}=zeros(2))
  #
  # W1 = randn(Float32, 4,8)
  # b1 = randn(Float32,8)
  modjl = Chain(
    x -> (x*W1)' .+ b1 .|> relu,
    x -> reshape(x', 25,8,1),
    x -> maxpool(x, PoolDims(x, 4)),
    # x -> reshape(x[:,:,1]',1,:),
    x -> reshape(x[:,:,1]',:),
    Dense(48,8,relu),
    Dense(8,2)
  )

  modjl[5].W .= W2
  modjl[5].b .= b2

  modjl[6].W .= W3
  modjl[6].b .= b3

  return modjl
end

# As loaded from tensorflow get_weights
# Super specialized function
function buildPyNNModel_01_FromWeights(pywe)
  buildPyNNModel_01_FromElements(pywe[1], pywe[2][:], pywe[3]', pywe[4][:], pywe[5]', pywe[6][:])
end


## test with lots of variability


testModJl = buildPyNNModel_01_FromWeights(pywe)

testModJl(jlX)




for k in 1:100
  MCpyX = [rand(4) for i in 1:25]
  # load data in near similar format
  MCjlX = zeros(25,4)
  for i in 1:25
    MCjlX[i,:] = MCpyX[i]
  end

  @show pymodel.predict([[MCpyX]])[:] - testModJl(MCjlX)[:] |> norm
  @assert pymodel.predict([[MCpyX]])[:] - testModJl(MCjlX)[:] |> norm < 1e-4
end






## Try consolidate the first dense layer

# julia> modTest1jl(jlX)
# ERROR: DimensionMismatch("A has dimensions (8,4) but B has dimensions (25,4)")


# load data in near similar format
# jlX = zeros(25,4)
# for i in 1:25
#   jlX[i,:] .= pyX[i]
# end

# ## test the first Dense layer
#
# W1 = pywe[1]
# b1 = pywe[2]
#
# Dense(4,8).W
#
# c1_1 = (jlX*W1)' .+ b1
#
# modTest1jl = Chain(
#   Dense(4,8),
# )
#
# modTest1jl[1].W .= pywe[1]'
# modTest1jl[1].b .= pywe[2]
#
# modTest1jl(jlX)
#
# # Build first portion of model to compare with Flux
#
# pm1_1 = pymodels.build_model1_1((25,4))
# pm1_1.set_weights(pywe[1:2])
# c1_1_py = pm1_1.predict([[pyX]])
#
# for i in 1:8
#   @assert c1_1_py[1,:,i] - c1_1[i,:] |> norm < 1e-4
# end
