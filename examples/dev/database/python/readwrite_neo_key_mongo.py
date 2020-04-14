# Fetch MongoKeys per key frame and re-insert
import cv2
import numpy as np
import os
from neo4j.v1 import GraphDatabase, basic_auth # via queries
from pymongo import MongoClient
import pymongo
from bson import Binary, json_util, BSON, ObjectId
import bson
import json # for



authfileneo = os.path.join(os.getenv('HOME'), 'neo_authfile.txt') # username on one line, password on next (for database)
un,pw,addrn = open(authfileneo).read().splitlines()

driver = GraphDatabase.driver(addrn, auth=basic_auth(un, pw))
session = driver.session()

# update and repeat this
above=0
recs = session.run("match (n:SESSTURTLE:POSE) where id(n) >"+str(above)+" return id(n) as id, n.mongo_keys as mongokeys order by id(n) asc limit 1")

# iterate through the poses and get the mongokeys
for rec in recs:
  print rec["id"], rec["mongokeys"]
  above=rec["id"]



#####################################################################################
## MongoDB Construction in Python Example
# Add a binary image entry (referred to as a "document") to a "collection" in a mongo database.
#####################################################################################

authfilemongo = os.path.join(os.getenv('HOME'), 'mongo_authfile.txt') # username on one line, password on next (for database)
addrm = open(authfilemongo).read().splitlines()
client = MongoClient(addrm)

print "------- GETTING From CG's PNG data ----------"
print ""


#USING NEW DATA
# connect to database using client
db = client.CloudGraphs # tester==name of the db. also db = client['tester'] works.
# db = client.tester # tester==name of the db. also db = client['tester'] works.

collection = "bindata"
# collection = "rgb_keyframes"


# New PNG dataset
key = ObjectId("58ab6b015d76250947c22498")
# key = ObjectId("589b7535093d3f1d51cc81da")

results = db[collection].find({"_id":key})

# get the image?
for item in results:
  im = cv2.imdecode(np.fromstring(str(item['val']), dtype=np.uint8), -1)
  print im.shape
