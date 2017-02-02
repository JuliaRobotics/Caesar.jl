from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json
import random#.uniform as ru

authfile = '/home/rmata/neo_authfile.txt' # username on one line, password on next (for database)
un,pw = open(authfile).read().splitlines()

#####################################################################################
## Database Construction in Python Example

## Number of odometry nodes:
NUM_ODOM_NODES = 5


#####################################################################################

# start up session
driver = GraphDatabase.driver("bolt://mrg-liljon.csail.mit.edu", auth=basic_auth(un, pw))
session = driver.session()

# clear SESSROX db each test...
session.run("MATCH (n:SESSROX) DETACH DELETE n")

for i in range(NUM_ODOM_NODES):
    old_odom_num = i
    curr_odom_num = i+1

    # generate odometry constraint between old_odom_num and curr_odom_num
    add_landmark = random.randint(0,1) # whether or not to add landmark to a certain odom
    loc = random.uniform(old_odom_num, curr_odom_num) # distance between them: goes into factor

    # encode JSON string to add to slamstring property of new4j node
    var_b4jso1 = {'odom_num':"ODOM " + str(old_odom_num)}
    var_b4jso2 = {'odom_num':"ODOM " + str(curr_odom_num)}
    var_json_str1 = json.dumps(var_b4jso1) 
    var_json_str2 = json.dumps(var_b4jso2)

    fac_b4jso = {'slam_info':"ODOM " + str(old_odom_num) + " " + str(curr_odom_num) + " " + str(loc)}
    fac_json_str = json.dumps(fac_b4jso)

    if old_odom_num == 0:
        session.run("MERGE (o1:POSE:ODOM:SESSROX {slam_info: {var_info1} }) " # finds, creates if not exist
                    "MERGE (f:FACTOR:SESSROX { slam_info: {fac_info} }) " # prior factor
                    "MERGE (o1)-[:PRIOR]-(f)",
                    {"var_info1": var_json_str1, 
                     "fac_info":json.dumps({"prior_dist":"prior distribution etc"})})
    
    # create nodes (if not already): property/key "slamstring", value JSON string? 
    session.run("MERGE (o1:POSE:ODOM:SESSROX {slam_info: {var_info1} }) " # finds, creates if not exist
                "MERGE (o2:POSE:ODOM:SESSROX { slam_info: {var_info2} })"
                "MERGE (f:FACTOR:SESSROX { slam_info: {fac_info} }) " # odom-odom factor
                "MERGE (o1)-[:REL]->(f) " # add relationships
                "MERGE (o2)-[:REL]->(f) ",
                {"var_info1":var_json_str1, 
                 "var_info2":var_json_str2, 
                 "fac_info":fac_json_str})

    if add_landmark:
        landmark_num = random.randint(0,15)
        landmark_old_odom_loc = random.uniform(min(old_odom_num, landmark_num), 
                                               max(old_odom_num, landmark_num)) 
        # measure between them: goes into factor

        fac_b4jso_land = {'land_info':"ODOM " + str(old_odom_num) + 
                          " LAND " + str(landmark_num) + " " + 
                          str(landmark_old_odom_loc)} # whatever the odom-factor should contain 
        land_fac_json_str = json.dumps(fac_b4jso_land)
        land_var_json = json.dumps({"tag_id":landmark_num})
        # add observation/new location var to current odom (IE new factor node, new POSE)
        session.run("MERGE (l1:POSE:LAND:SESSROX {slam_info: {land_info}})"
                    "MERGE (o1:POSE:ODOM:SESSROX {slam_info: {var_info}})"
                    "MERGE (f:FACTOR:SESSROX {slam_info: {fac_info}})" # odom-landmark factor
                    "MERGE (o1)-[:REL]->(f) "
                    "MERGE (f)-[:REL]->(l1) ",
                    {"land_info": land_var_json, 
                     "var_info": var_json_str1, 
                     "fac_info": land_fac_json_str})
    
odom_results = session.run("MATCH (n:ODOM:SESSROX) RETURN n")
factor_results = session.run("MATCH (n:FACTOR:SESSROX) RETURN n")
landmark_results = session.run("MATCH (n:LAND:SESSROX) RETURN n")
for record in odom_results:
    print record
for record in factor_results:
    print record
for record in landmark_results:
    print record

