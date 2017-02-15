# make copy of current db session

"""
match (n:SESSROX)
remove n.backendset, n.ready, n.data, n.bigData, n.label, n.packedType, n.exVertexId, n.shape, n.width
set n :NEWDATA
return n
"""











#
