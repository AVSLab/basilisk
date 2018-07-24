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
//
// TcpClient.h
//
//

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "BasicIoDevice.h"
#include <stdint.h>

class TcpClient
    : public BasicIoObject_t<boost::asio::ip::tcp::socket>
{
public:
    TcpClient(boost::asio::io_service *ioService);

    int connect(std::string ipAddress = "127.0.0.1",
                uint32_t portNum = 50000);

    bool receiveData(std::vector<char> &data);
    bool sendData(std::vector<char> & data);
    
    bool close(void);

    virtual void clearBuffers(void);
};

#endif
