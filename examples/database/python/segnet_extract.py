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

from neo4j.v1 import GraphDatabase, basic_auth
from pymongo import MongoClient

from bson import Binary, json_util, BSON, ObjectId
import json

from pybot.utils.io_utils import create_path_if_not_exists
from pybot.vision.image_utils import im_resize, to_color, to_gray
sys.path.append(os.path.join(os.getenv('PYCAFFE'), 'python'))

from pybot.vision.caffe import setup_segnet

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
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Segnet directory')
    parser.add_argument(
        '-d', '--directory', type=str, default='', required=True, 
        help='Segnet directory')
    args = parser.parse_args()
    
    # username on one line, password on next (for database)
    authfileneo = os.path.join(os.getenv('HOME'), 'neo_authfile.txt') 
    un, pw, addrn = open(authfileneo).read().splitlines()

    # Graph DB driver
    driver = GraphDatabase.driver(addrn, auth=basic_auth(un, pw))
    session = driver.session()

    # update and repeat this
    above = 0
    recs = session.run('''match (n:SESSTURTLE:POSE) where id(n) > {} '''
                       '''return id(n) as id, n.mongo_keys as mongokeys, '''
                       '''n.frtend as label order by id(n) asc limit 1'''.format(above))

    # iterate through the poses and get the mongokeys
    for index in range(0,9): 
        for rec in recs:
          print 'record', rec["id"], rec["label"], rec["mongokeys"]
          above = rec["id"]
          recs = session.run('''match (n:SESSTURTLE:POSE) where id(n) > {} '''
                             '''return id(n) as id, n.mongo_keys as mongokeys, '''
                             '''n.frtend as label order by id(n) asc limit 1'''.format(above))


    #####################################################################################
    ## MongoDB Construction in Python Example
    # Add a binary image entry (referred to as a "document") to a "collection" in a mongo database.
    #####################################################################################

    authfilemongo = os.path.join(os.getenv('HOME'), 'mongo_authfile.txt') # username on one line, password on next (for database)
    addrm = open(authfilemongo).read().splitlines()
    client = MongoClient(addrm)

    print "------- GETTING From CG's PNG data ----------"
    print ""


    # USING NEW DATA
    # connect to database using client
    db = client.CloudGraphs # tester==name of the db. also db = client['tester'] works.
    # db = client.tester # tester==name of the db. also db = client['tester'] works.

    collection = "bindata"
    # collection = "rgb_keyframes"

    # New PNG dataset
    key = ObjectId("58ab6b015d76250947c22498")
    # key = ObjectId("589b7535093d3f1d51cc81da")

    results = db[collection].find({"_id":key})

    # Setup segnet
    segnet = SegnetExtractor(os.path.expanduser(args.directory))

    # get the image?
    for item in results:
      im = cv2.imdecode(np.fromstring(str(item['val']), dtype=np.uint8), -1)

      for j in range(100): 
          label = segnet.extract(im)
          print im.shape, label.shape
