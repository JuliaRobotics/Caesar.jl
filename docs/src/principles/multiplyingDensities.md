# Principle: Multiplying Functions

This principle illustration shows one of the core operations used in Caesar.jl (multimodal-iSAM), where different probability belief functions are multiplied together.
The true product between various likelihood beliefs is very complicated to compute, but a good approximations exist.
In addition, `ZmqCaesar` offers a `ZMQ` interface to the factor graph solution for multilanguage support.  This example is a small subset that shows how to use the `ZMQ` infrastructure, but avoids the larger factor graph related calls.

## Starting the ZMQ server

Caesar.jl provides a [startup script for a default ZMQ instance](https://github.com/JuliaRobotics/Caesar.jl/blob/master/scripts/zmqServer.sh).  Start a server and allow precompilations to finish until a printout message "waiting to receive..." is seen.  Feel free to change the ZMQ interface for TCP vs. shared memory or any of the ZMQ supported modes of data transport.

## Functional Products via Python

Clone the Python [`GraffSDK.py` code here](http://github.com/nicrip/graff_py) and find the `test.py` file.
```python
import numpy as np
from graff.Endpoint import Endpoint
from graff.Distribution.Normal import Normal
from graff.Distribution.SampleWeights import SampleWeights
from graff.Distribution.BallTreeDensity import BallTreeDensity

from graff.Core import MultiplyDistributions

import matplotlib.pyplot as plt

if __name__ == '__main__':
    e = Endpoint()

    e.Connect('tcp://192.168.0.102:5555')
    print(e.Status())

    N = 100
    u1 = 0.0
    s1 = 3.0

    u2 = 50.0
    s2 = 3.0
    b1 = BallTreeDensity('Gaussian', np.ones(N), np.ones(N), u1+s1*np.random.randn(N))
    b2 = BallTreeDensity('Gaussian', np.ones(N), np.ones(N), u2+s2*np.random.randn(N))

    rep = MultiplyDistributions(e, [b1,b2])
    print(rep)
    x = np.array(rep['payload']['points'] )
    # plt.stem(x, np.ones(len(x)) )
    plt.hist(x, bins = int(len(x)/10.0))
    plt.show()

    e.Disconnect()
```


# A Basic Factor Graph Product Illustration
