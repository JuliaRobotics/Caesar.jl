from pymongo import MongoClient
import pymongo
from bson import Binary, json_util, BSON
import bson
import numpy as np

class mongo_interact(object):
    ''' Non-parametric SLAMinDB Neo4j interface '''
    def __init__(self, authfile, session_name, collection_name):
        self._authfile = authfile
        self._session_name = session_name
        self._collection_name = collection_name

        # initialize mongo session
        self.DB_address = open(self._authfile).read().splitlines()
        print self.DB_address
        client = MongoClient(self.DB_address)
        self.db = client.CloudGraphs

    def insert_array(self, data):
        if len(data.shape) == 1:
            r = data.shape[0]
            c = 1
        else:
            r,c = data.shape
        bin_data = Binary(data.tostring())
        key = self.db[self._collection_name].insert({"neoNodeId": -1,"rows":r, "cols":c, "val": bin_data, "description": "Auto-inserted with mongo_interact.py"})
        return key