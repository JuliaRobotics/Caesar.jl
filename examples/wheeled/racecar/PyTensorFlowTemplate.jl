# load required tensor flow functionality

ENV["PYTHON"] = "/usr/bin/python3.6"

using Pkg
Pkg.build("PyCall")

using PyCall

tf = pyimport("tensorflow")

tf.keras.models.load_model

Base.cd(ENV["USER"]*"/learning-odometry/")

mmodel = tf.keras.models.load_model("original_network0.h5")

mmodel.predict

A = [rand(4) for i in 1:25]

mmodel.predict([[A]])
# 1×2 Array{Float32,2}:
#  0.0169379  1.58055


models = []
0-element Array{Any,1}

using ProgressMeter

@showprogress "loading models" for i in 0:9
  push!(models, tf.keras.models.load_model("original_network$i.h5"))
end

models[1].predict([[A]])

preds = map(x->x.predict([[A]]), models)


predsingle = pyimport("queryAllModels")["predictSinglePose"]

predsingle(A)
# returns  10×3 Array{Real,2}:
