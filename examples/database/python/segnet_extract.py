"""

Run as: 
export PYCAFFE=$HOME/code/caffe-segnet/; python segnet_extract.py -d ~/software/segnet-tutorial

"""

# Fetch MongoKeys per key frame and re-insert
import os
import sys
import argparse
import cv2
import numpy as np
import time

from neo4j.v1 import GraphDatabase, basic_auth
from pymongo import MongoClient

from bson import Binary, json_util, BSON, ObjectId
import json

from pybot.utils.io_utils import create_path_if_not_exists
from pybot.vision.image_utils import im_resize, to_color, to_gray

sys.path.append(os.path.join(os.getenv('PYCAFFE'), 'python'))

class SegnetExtractor(object):
    def __init__(self, directory):

        segnet_model = os.path.join(directory, 'Example_Models/segnet_sun.prototxt')
        segnet_weights = os.path.join(directory, 'Example_Models/segnet_sun.caffemodel')
        
        # Color LUT
        self.colors = cv2.imread(os.path.join(directory, 'Models/sun.png')).astype(np.uint8)
        self.segnet = setup_segnet(segnet_model, segnet_weights)

    def extract(self, im):
        self.segnet.forward(im)
        labels = self.segnet.extract(layer='argmax')

        # Resize labels to image
        out = labels.astype(np.uint8)
        out = im_resize(out, shape=im.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
        cout = np.dstack([out, out, out])
        colored = cv2.LUT(cout, self.colors)

        return colored        
    

def updatemongokeys(session, dbsessname, neoid, oidname, newoid):
    res = session.run("MATCH (n:"+dbsessname+") " # finds if not exist
                      "WHERE id(n)="+str(neoid)+" "
                      "return n.frtend as frtend, n.mongo_keys as mongo_keys")

    elem = res.single()
    var_json_str1 = elem["frtend"]
    mymoids = elem["mongo_keys"]
    if mymoids == None:
        mymoids = {}
    else:
        mymoids = json.loads(mymoids)

    mymoids[oidname] = newoid
    mym_json_str = json.dumps(mymoids)

    # import IPython; IPython.embed()
    
    res = session.run("MATCH (n:"+dbsessname+") "
                      "WHERE id(n)="+str(neoid)+" "
                      "SET n.mongo_keys = {mym_info} "
                      "RETURN count(n) as numupdated",
                      {"mym_info": mym_json_str} )

    elem = res.single()
    if elem["numupdated"] != 1:
        print "ERROR, did not work. Number of updated entries are", elem["numupdated"]
    # res = session.run("MERGE (o1:POSE:SESSROX { frtend: {var_info1} }) "
    #                   "set o1.mongo_keys = {mym_info} "
    #                   "return o1.frtend as frtend, o1.mongo_keys as mongo_keys ",
    #                   {"var_info1": var_json_str1,
    #                    "mym_info": mym_json_str})


    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Segnet directory')
    parser.add_argument(
        '-d', '--directory', type=str, default='', required=True, 
        help='Segnet directory')
    parser.add_argument(
        '-g', '--gpu', type=int, default=0, required=True, 
        help='GPU ID')
    parser.add_argument(
        '-S', '--session', type=str, default='', required=True, 
        help='Session name')
    args = parser.parse_args()
    print('Using session {}'.format(args.session))
    print('Using GPU {}'.format(args.gpu))

    import caffe
    caffe.set_device(args.gpu)
    from pybot.vision.caffe import setup_segnet
    
    # username on one line, password on next (for database)
    authfileneo = os.path.join(os.getenv('HOME'), 'neo_authfile.txt') 
    un, pw, addrn = open(authfileneo).read().splitlines()

    # Graph DB driver
    driver = GraphDatabase.driver(addrn, auth=basic_auth(un, pw))
    session = driver.session()

    # Mongo client
    authfilemongo = os.path.join(os.getenv('HOME'), 'mongo_authfile.txt') # username on one line, password on next (for database)
    addrm = open(authfilemongo).read().splitlines()
    client = MongoClient(addrm)
    db = client.CloudGraphs
    collection = "bindata"
    
    # connect to database using client
    # db = client.tester # tester==name of the db. also db = client['tester'] works.

    # Setup segnet
    segnet = SegnetExtractor(os.path.expanduser(args.directory))
    
    # iterate through the poses and get the mongokeys
    last_neo_id = 0
    for index in range(198):
        recs = session.run('''match (n:{}:POSE) where id(n) > {} '''
                           '''return id(n) as id, n.mongo_keys as mongokeys, '''
                           '''n.frtend as label order by id(n) asc limit 1'''.format(args.session, last_neo_id))
        for rec in recs:
          # print 'record', rec["id"], rec["label"], rec["mongokeys"]
          last_neo_id = rec["id"]
          try: 
              mongo_key = ObjectId(json.loads(rec['mongokeys'])['keyframe_rgb'])
              results = db[collection].find({"_id": mongo_key})
          except Exception, e:
              print('Skipping frame {}'.format(e))
              continue
          
          for item in results:
              im = cv2.imdecode(np.fromstring(str(item['val']), dtype=np.uint8), -1)
              label = segnet.extract(im)
              
              # Re-encode segnet label
              res, label_encoded = cv2.imencode('.png', label)
              label_monogo_key = db[collection].insert({"neoNodeId": -1, "val": Binary(label_encoded.tostring()),
                                                        "description": "Auto-inserted with mongo_interaction.py"})

              print 'New mongo key for segnet label', label_monogo_key, type(label_monogo_key)
              updatemongokeys(session, args.session, last_neo_id, 'keyframe_segnet', str(label_monogo_key))
              time.sleep(0.1)
          

    # #####################################################################################
    # ## MongoDB Construction in Python Example
    # # Add a binary image entry (referred to as a "document") to a "collection" in a mongo database.
    # #####################################################################################

    # print "------- GETTING From CG's PNG data ----------"
    # print ""


    # # collection = "rgb_keyframes"

    # # New PNG dataset
    # # key = ObjectId("58ab6b015d76250947c22498")
    # # key = ObjectId("589b7535093d3f1d51cc81da")


    # # get the image?

