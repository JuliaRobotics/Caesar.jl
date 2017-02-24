# python read function file

import bson
import pymongo
import cv2
import numpy as np

def getrgbimg(addrm, collection, key):
    mongo_key = bson.ObjectId(key)
    client = pymongo.MongoClient(addrm)
    db = client.CloudGraphs
    results = db[collection].find({"_id": mongo_key})
    im = []
    for item in results:
      im = cv2.imdecode(np.fromstring(str(item['val']), dtype=np.uint8), -1)
    return im.shape

def fastrgbimg(dbcoll, mongo_key):
    results = dbcoll.find({"_id": mongo_key})
    im = None
    for item in results:
      im = cv2.imdecode(np.fromstring(str(item['val']), dtype=np.uint8), -1)
    return im, im.shape

def fastdepthimg(dbcoll, mongo_key):
    results = dbcoll.find({"_id": mongo_key})
    im = None
    for item in results:
      im  = np.fromstring(str(item['val']), dtype=np.float32).reshape(480,640)
    return im, im.shape

def test(key, val):
    mongo_key = bson.ObjectId(key)
    return mongo_key






# console example

# import bson
# import pymongo
# import cv2
# import numpy as np
#
# addrm =
# collection = "bindata"
#
# keydepth = "58b07be05d7625699151a01d"
#
# mongo_key = bson.ObjectId(keydepth)
# client = pymongo.MongoClient(addrm)
# db = client.CloudGraphs
#
# results = db[collection].find({"_id": mongo_key})
# for item in results:
#     im  = np.fromstring(str(item['val']), dtype=np.float32).reshape(480,640)
#     # print item["val"]
#     # im = cv2.imdecode(np.fromstring(str(item['val'])), -1)
#
# print im.shape, im.dtype, type(im)
#
# # print np.dtype(im)
#
# cv2.imshow('depth', im)













#
