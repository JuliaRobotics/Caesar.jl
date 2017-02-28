# add specific loop closure using Odo constraint
# yes, needs to be optimized and refactored

using IncrementalInference, CloudGraphs


## Uncomment out for command line operation
cloudGraph, addrdict = standardcloudgraphsetup()
session = addrdict["session"]

# interactive operation
# session = "SESSROX"
# include(joinpath(dirname(@__FILE__),"blandauthremote.jl"))


# Register protobuf types of interest in CloudGraphs layer
registerGeneralVariableTypes!(cloudGraph)

fg = emptyFactorGraph()
fg.cg = cloudGraph

fullLocalGraphCopy!(fg)

v1a = getVert(fg, :l9)
v1b = getVert(fg, :l16)

f1 = addFactor!(fg,[v1a;v1b],Odo(zeros(2,1),0.1*eye(2),[1.0]),ready=0)

setDBAllReady!(conn)



# x144, x109 and x15, 161 and x115, x137 and x122, 132 and x131, 123
# x37, x149 and x113, 139 and x112,141 and x17,x160, and x34,x154
# x119, x135 and x10, x175 and x38, x148 and x8, x181 and x21, x157
v2a = getVert(fg, :x21)
v2b = getVert(fg, :x157)

DX = zeros(3,1)
DX[3] = pi
pp = Pose2Pose2(DX, 0.1*eye(3), [1.0]) #[prev;v],
f = addFactor!(fg, [v2a;v2b], pp, ready=0)

setDBAllReady!(conn)


# DX = zeros(3)
# x164, x170


# 9/14 l4, l7 and l5, l6 and l3, l11 and l3, l13 and

# 8/22 l1, l6 and l1 and l10
