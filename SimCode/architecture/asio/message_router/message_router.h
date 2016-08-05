/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>


class MessageRouter: public SysInterface {
public:
    MessageRouter();
    ~MessageRouter();
    void initializeServer(std::string hostName, uint32_t portStart);
     
public:
    
private:
    boost::asio::io_service ioService;
    TcpServer *serverConnection;
    TcpClient *clientConnection;
};

#endif
