using Mongo, LibBSON

client = MongoClient("mrg-liljon.csail.mit.edu", 27017)
cgBindataCollection = MongoCollection(client, "CloudGraphs", "bindata")









# Example query
# does not deal with wrap around issue yet
n.MAP_est = [x; y; yaw]
setp = [x; y]

match (n)
where (n.MAP_est[0] - setp[0])*(n.MAP_est[0] - setp[0]) + (n.MAP_est[1] - setp[1])*(n.MAP_est[1] - setp[1]) < 10.0
			and
			abs( atan2( (0-n.MAP_est[1]), (0-n.MAP_est[0]) ) - n.MAP_est[2] ) < 0.3
return n
