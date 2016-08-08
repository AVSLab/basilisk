//
// TcpClient.h
//
//

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "basicIoDevice.h"
#include <stdint.h>

class TcpClient
    : public BasicIoObject_t<boost::asio::ip::tcp::socket>
{
public:
    TcpClient(boost::asio::io_service *ioService);

    int connect(std::string ipAddress = "127.0.0.1",
                uint32_t portNum = 50000);

    virtual bool receiveData(std::vector<char> &data);
    virtual bool sendData(std::string data);

    virtual void clearBuffers(void);
};

#endif