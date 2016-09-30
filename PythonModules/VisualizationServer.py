import threading
import SocketServer
import VisualizationDataDelegate
import time
from pudb import set_trace

class VisualizationServer:
    def __init__(self, simulation):
        # a reference to the simulation is passed the dataSource
        # a class member variable for both dataSource and dataDelegate
        # is here for clarity and potential manipulation from the simulation level
        host_ip, port = "127.0.0.1", 50000
        self.server = ThreadedTCPServer((host_ip, port), ThreadedTCPRequestHandler)
        self.server.request_queue_size = 1
        # self.server.dataSource = VisualizationDataSource.DataSource(simulation)
        self.server.dataDelegate = VisualizationDataDelegate.DataDelegate(simulation)


    def startServer(self):
        ip, port = self.server.server_address
        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        serverThread = threading.Thread(target=self.server.serve_forever)
        # Exit the server thread when the main thread terminates
        serverThread.setDaemon(False)
        serverThread.start()
        print "Server loop running in thread:", serverThread.name


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    allow_reuse_address = True
    pass


class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    def __init__(self, request, client_address, server):
        self.requestData = None
        self.BUFFER_SIZE = 4096
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)
        return

    def handle(self):
        # set_trace()
        while True:
            # print "handle"+str(threading.current_thread())
            self.requestData = self.request.recv(self.BUFFER_SIZE)
            requestPayloadSize = self.requestData[:8]
            requestType = self.requestData[8:10]

            if len(self.requestData) == self.BUFFER_SIZE: #or possibly == to payload size:
                while True:
                    try:  # error means no more data
                        self.requestData += self.request.recv(self.BUFFER_SIZE)
                    except:
                        break
                        # no data found exit loop (posible closed socket)
            if self.requestData == "":
                break

            # print str(self.client_address[0])+" wrote: "
            # print "Received:"
            # print self.requestData

            self.sendResponse(requestType)

    def sendResponse(self, requestType):
        # print int(time.time())
        responseData = []
        if requestType == "00":
            responseData = self.server.dataDelegate.packageData()
            encodedData = self.server.dataDelegate.encodeDataToJSON(responseData)
            print encodedData
        elif requestType == "01":
            print requestType
        else:
            print 'requestType: ' + requestType + ' unknonwn'

        if encodedData != "[]":
            sentStuff = self.request.sendall(str(len(encodedData))+encodedData)
            if sentStuff is None:
                print "Send Successful"
        else:
            sentStuff = self.request.sendall("0011EmptyString")
