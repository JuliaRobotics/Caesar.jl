from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json
import random#.uniform as ru

#authfile = open('/home/rmata/authfile.txt')
un,pw = open('/home/rmata/neo_authfile.txt').read().splitlines()
print un
print pw
#pw = None

# query example
driver = GraphDatabase.driver("bolt://mrg-liljon.csail.mit.edu", auth=basic_auth(un, pw))
session = driver.session()

# clear SESSROX each test...
session.run("MATCH (n:SESSROX) DETACH DELETE n")

for i in range(10):
    old_odom_num = i
    curr_odom_num = i+1
    loc = random.uniform(old_odom_num, curr_odom_num)
    # encode JSON string to add to slamstring property of new4j node
    b4jso = {'name':"ODOM " + str(old_odom_num) + " " + str(curr_odom_num) + " " + str(loc)}
    json_str = json.dumps(b4jso)
    
    # create nodes: property/key "slamstring", value JSON string? 
    session.run("CREATE (n:POSE:SESSROX { slam_info: {info} } )", {"info":json_str}) 
    
result = session.run("MATCH (n:POSE:SESSROX) return n")
for record in result:
    print record#("%s %s" % (record["name"]))
    
