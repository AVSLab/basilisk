//
// TcpServer.h
//
//

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "basicIoDevice.h"

class TcpServer
    : public BasicIoObject_t<boost::asio::ip::tcp::socket>
{
public:
    TcpServer(boost::asio::io_service *ioService);

    bool acceptConnections(std::string ipAddress = "127.0.0.1",
                           std::string portNum = "50000");
    virtual bool close(void);

    virtual bool receiveData(std::vector<char> &data);
    virtual bool sendData(std::string data);

    virtual void clearBuffers(void);

private:
    boost::scoped_ptr<boost::asio::ip::tcp::acceptor> m_acceptor;
};

#endif