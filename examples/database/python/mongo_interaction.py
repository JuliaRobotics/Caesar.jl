from pymongo import MongoClient
import pymongo
from bson import Binary, json_util, BSON
import bson
import cv2 # for image pulling/read in only
import json # for

#####################################################################################
## MongoDB Construction in Python Example
# Add a binary image entry (referred to as a "document") to a "collection" in a mongo database.
#####################################################################################

authfile = '/home/dehann/mongo_authfile.txt' # username on one line, password on next (for database)
addr = open(authfile).read().splitlines()
print addr # address should be "mongodb://username:password@host1:port"

client = MongoClient(addr)

# connect to database using client
db = client.CloudGraphs # tester==name of the db. also db = client['tester'] works.
collection = "bindata"

# find image, make binary/BSON, ready for insertion
## NOTE this assumes image is under 16 MB. If it is over, use GridFS for inserting into Mongo
X = cv2.imread("/home/dehann/Downloads/sample_image.jpg")
print type(X)
f = BSON(X) # blah 6Kb image of a printer


# insert!
key = db[collection].insert({"neoNodeId": -1, "val": f, "description": "Auto-inserted with mongo_interaction.py"})

# check that mongo ate your key/image combo by asking it for image via key ref
#  - start up mongo
#                         $ mongo
#  - check that your database is there
#                         $> show dbs
#  - switch to the database
#                         $> use tester
#  - check that the collection is there
#                         $> show collections
#  - find the blob that the key references
#                         $> db.testing_collection.find({"key":<key>})
#  - want to drop/forget/erase(?) a collection?
#                         $> db.testing_collection.drop()

# get the image?
for result in db.bindata.find({"_id":key}):
    newf = open('/home/dehann/newfile.png', 'w') # to write image to
    #print type(result["image"])
    X = BSON(result["val"]).decode()
    newf.close()
