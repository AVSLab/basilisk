''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import threading
import SocketServer
import VisualizationDataDelegate
import time

class VisualizationServer:
    def __init__(self, simulation):
        # a reference to the simulation is passed the dataSource
        # a class member variable for both dataSource and dataDelegate
        # is here for clarity and potential manipulation from the simulation level
        host_ip, port = "127.0.0.1", 50000
        self.server = ThreadedTCPServer((host_ip, port), ThreadedTCPRequestHandler)
        self.server.request_queue_size = 5
        # self.server.dataSource = VisualizationDataSource.DataSource(simulation)
        self.server.dataDelegate = VisualizationDataDelegate.DataDelegate(simulation)

    def startServer(self):
        ip, port = self.server.server_address
        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        serverThread = threading.Thread(target=self.server.serve_forever)
        # Exit the server thread when the main thread terminates
        serverThread.setDaemon(True)
        serverThread.start()
        print "Server loop running in thread:", serverThread.name


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    allow_reuse_address = True
    pass


class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    def __init__(self, request, client_address, server):
        self.requestData = None
        self.BUFFER_SIZE = 2048
        # self.timing = []
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)
        return

    def handle(self):
        while True:
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
        responseData = []
        if requestType == "00":
            responseData = self.server.dataDelegate.packageData()
            # t1 = time.clock()

            encodedData = self.server.dataDelegate.encodeDataToJSON(responseData)
            # encodedData = ujson.dumps(responseData)
            # t2 = time.clock()
            # self.timing.append(t2-t1)
            # print sum(self.timing)/len(self.timing)

        elif requestType == "01":
            print requestType
        else:
            print 'requestType: ' + requestType + ' unknonwn'

        sentStuff = self.request.sendall(str(len(encodedData))+encodedData)
        # if encodedData != "[]":
        #     # t1 = time.clock()
        #     sentStuff = self.request.sendall(str(len(encodedData))+encodedData)
        #     # t2 = time.clock()
        #     # print "send time: " + str(t2-t1)
        #     if sentStuff is not None:
        #         print "Send data failed"
        # else:
        #     sentStuff = self.request.sendall("0011EmptyString")