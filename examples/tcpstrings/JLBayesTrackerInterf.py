import json
import socket
import sys

def readlines(sock, recv_buffer=4096, delim='\n'):
	buffer = ''
	data = True
	while data:
		data = sock.recv(recv_buffer)
		buffer += data
		while buffer.find(delim) != -1:
			line, buffer = buffer.split('\n', 1)
			yield line
	return

class BayesFeatureTracking(object):
    def __init__(self):
        self.cl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print 'Created BayesFeatureTracking object'

    def connectServer(self, addr='localhost', port=60002):
        # Connect the socket to the port where the server is listening
        self.server_address = (addr, port)
        print >>sys.stderr, 'connecting to %s port %s' % self.server_address
        self.cl.connect(self.server_address)
        print 'BayesFeatureTracking object connected to ', addr, port

    def disconnectServer(self):
        self.cl.close()
        print 'BayesFeatureTracking object disconnecting from server'

    def sendDictJSON(self, d):
        msg = json.dumps(d)+'\n'
        self.cl.sendall(msg)
        if d['CMD'] == 'QUIT':
            self.disconnectServer()
        else:
            for line in readlines(self.cl):
                return line

    def close(self):
        d = {'CMD' : 'QUIT'}
        self.sendDictJSON(d)

    def processSightings(self, dOdo, r):
        d = {'CMD' : 'PROCESS'}
        d['dx'] = dOdo
        d['BRfeats'] = r
        resp = self.sendDictJSON(d)
        retd = json.loads(resp)
        print 'response', retd
