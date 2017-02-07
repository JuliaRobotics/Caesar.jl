from pymongo import MongoClient
import pymongo
from bson import Binary
import uuid
import pickle
import cv2 # for image pulling/read in only
#impot json # for

#####################################################################################
## MongoDB Construction in Python Example
# Add a binary image entry (referred to as a "document") to a "collection" in a mongo database.
#####################################################################################

authfile = '/home/rmata/mongo_authfile.txt' # username on one line, password on next (for database)
addr = open(authfile).read().splitlines()
print addr # address should be "mongodb://username:password@host1:port"

client = MongoClient(addr)

# connect to database using client
db = client.tester # tester==name of the db. also db = client['tester'] works.
collection = "testing_collection"

# find image, make binary/BSON, ready for insertion
## NOTE this assumes image is under 16 MB. If it is over, use GridFS for inserting into Mongo
X = cv2.imread("/home/rmata/Downloads/sample_image.jpg")
f = Binary(pickle.dumps(X)) # blah 6Kb image of a printer

# create key (uuid straightup doesn't work) (extra key like this might be redundant)
key = str(uuid.uuid4().hex)
print "key is: ", key

# insert!
db[collection].insert({"key": key,
                       "image": f})

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
