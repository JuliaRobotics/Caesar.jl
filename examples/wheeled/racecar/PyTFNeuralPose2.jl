# PyTFNeuralPose2

ENV["PYTHON"] = "/usr/bin/python3.6"

using Pkg
Pkg.build("PyCall")

using PyCall

tf = pyimport("tensorflow")

tf.keras.models.load_model

Base.cd(ENV["USER"]*"/learning-odometry/")







#
