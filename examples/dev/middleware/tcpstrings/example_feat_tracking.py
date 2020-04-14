
# at one terminal run $ julia -p10 -e "using Caesar; tcpStringBRTrackingServer();"

from JLBayesTrackerInterf import BayesFeatureTracking


bft = BayesFeatureTracking()
bft.connectServer()

for i in range(3):
    r = [[10,0.8,1], [20,0,1]]
    dx = [0,0,0]
    bft.processSightings(dx, r)
r = [[19,0,1],[9.5,0.85,1],[17,-0.4,1]]
dx = [1,0,0]
bft.processSightings(dx, r)

bft.close()
print 'done'
