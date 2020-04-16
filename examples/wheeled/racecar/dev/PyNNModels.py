import csv
import json
import tensorflow as tf
import numpy as np
import dataPreProcLoopVersion as data
import matplotlib.pyplot as plt
from tensorflow.keras.models import load_model
#import keras.backend as K
import copy
from tensorflow.keras.callbacks import History
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Conv2D, Dense, Flatten, Input, MaxPooling2D, MaxPooling1D, Layer
from tfdiffeq.models import ODENet#, ConvODENet

history = History()

np.set_printoptions(threshold=np.inf)

# tf.enable_eager_execution()

bad_data_count = 0

def build_model(input_shape):
    x = Input(input_shape)
    y = Dense(8, activation=tf.nn.relu)(x)
    y = MaxPooling1D(4)(y)
    y = Flatten()(y)
    #y = ODENet(10, 1)(y)
    y = Dense(8, activation=tf.nn.relu)(y)
    y = Dense(2)(y)
    return Model(x, y)

# model = build_model((25,4))

def build_model1_1(input_shape):
    x = Input(input_shape)
    y = Dense(8)(x)
    return Model(x, y)

def build_model1(input_shape):
    x = Input(input_shape)
    y = Dense(8, activation=tf.nn.relu)(x)
    return Model(x, y)

def build_model2(input_shape):
    x = Input(input_shape)
    y = Dense(8, activation=tf.nn.relu)(x)
    y = MaxPooling1D(4)(y)
    return Model(x, y)

def build_model3(input_shape):
    x = Input(input_shape)
    y = Dense(8, activation=tf.nn.relu)(x)
    y = MaxPooling1D(4)(y)
    y = Flatten()(y)
    return Model(x, y)
