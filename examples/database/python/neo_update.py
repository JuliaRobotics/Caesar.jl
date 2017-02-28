
#####################################################################################
## Neo4j database update in example with Python
#####################################################################################

from neo4j.v1 import GraphDatabase, basic_auth # via queries
import json

authfile = '/home/dehann/neo_authfile.txt' # username on one line, password on next (for database)
un,pw,addr = open(authfile).read().splitlines()
print un, pw, addr


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
        print 'trying to load json'
        mymoids = json.loads(mymoids)

    mymoids[oidname] = newoid
    mym_json_str = json.dumps(mymoids)

    res = session.run("MATCH (n:POSE:SESSROX) "
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





# start up session
driver = GraphDatabase.driver(addr, auth=basic_auth(un, pw))
session = driver.session()


# say which node you want to update and how
dbsessname = "SESSROX"
neoid = 110728


oidname = "new-oid"
newoid = 1234


updatemongokeys(session, dbsessname, neoid, oidname, newoid)























#
