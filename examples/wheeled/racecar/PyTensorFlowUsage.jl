# load required tensor flow functionality

ENV["PYTHON"] = "/usr/bin/python3.6"

using Pkg
Pkg.build("PyCall")

using PyCall

# tf = pyimport("tensorflow")
# tf.keras.models.load_model

py"""
import sys
sys.path.insert(0, ENV["HOME"]*"/learning-odometry/")
"""

# @show tfmodeldir = joinpath(ENV["HOME"],"learning-odometry/")
# Base.cd(tfmodeldir)

PyTFOdoPredictorPoint2 = pyimport("queryAllModels")["predictSinglePose"]

# test that its working
A = [rand(4) for i in 1:25]
PyTFOdoPredictorPoint2(A)
# returns  10×3 Array{Real,2}:


# mmodel = tf.keras.models.load_model(joinpath(tfmodeldir, "original_network0.h5"))
# mmodel.predict
# mmodel.predict([[A]])
# 1×2 Array{Float32,2}:
#  0.0169379  1.58055
