/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

 */

#ifndef MESSAGE_ROUTER_H
#define MESSAGE_ROUTER_H

#include <vector>
#include "_GeneralModuleFiles/sys_interface.h"
#include "../_GeneralModuleFiles/TcpClient.h"
#include "../_GeneralModuleFiles/TcpServer.h"
#include "architecture/messaging/system_messaging.h"
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <stdint.h>

typedef struct {
    char messageName[MAX_MESSAGE_SIZE];
    uint64_t messageID;
}MessageNameIDPair;

typedef struct {
    int64_t messageID;
    uint64_t lastWriteCounter;
} MessageWriteData;

typedef struct {
    int64_t messageID;
    SingleMessageHeader headerData;
} ExchangeMessageHeader;

class MessageRouter: public InterfaceDataExchange {
public:
    MessageRouter(BasicIoObject_t<boost::asio::ip::tcp::socket> *inConnection = nullptr);
    ~MessageRouter();
    MessageRouter(std::string from, std::string to, std::string intName="", BasicIoObject_t<boost::asio::ip::tcp::socket> *inConnection=nullptr);
    bool initializeServer(std::string hostName, uint32_t portStart);
    bool initializeClient(std::string hostName, uint32_t portServer);
    BasicIoObject_t<boost::asio::ip::tcp::socket> * getConnection() {return theConnection;}
    bool linkProcesses();
    void discoverMessages();
    void UpdateState(uint64_t CurrentSimNanos);
    void requestUnknownMessages();
    void receiveUnknownMessages();
    void routeMessages();
    void closeConnection();
     
public:
    bool runAsServer;
    bool blockConnection;     //! [-] Flag indicating whether connection should block
    uint32_t defaultPort;     //! [-] Portnumber to start with on server and the connect to with client
    std::string hostName;     //! [-] Name of the host that we are connecting to
    
private:
    boost::asio::io_service ioService;
    BasicIoObject_t<boost::asio::ip::tcp::socket> *theConnection;
    TcpServer *serverConnection;
    TcpClient *clientConnection;
    std::map<int64_t, MessageWriteData> routerMapping;
    bool processLinked;       
};

#endif
