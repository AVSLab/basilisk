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
// boost_communication.h
//
//

#ifndef _BOOST_COMMUNICATION_H_
#define _BOOST_COMMUNICATION_H_

#include <iostream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread.hpp>
#include "architecture/asio/visMessageInterface/TcpSerializeServer.h"
#include "SpacecraftSimDefinitions.h"

#define IP_BASE_PORT         50000
#define MAX_CONNECT_ATTEMPTS 5
#define SERIAL_BAUD_RATE     115200

typedef enum ComStatus_t {
    COM_INIT,
    COM_SHUTDOWN,
    COM_SEND,
    COM_RECEIVE,
    COM_ON_DEMAND_INIT,
    COM_ON_DEMAND_POLL,
    MAX_COM_STATUS
} ComStatus_t;


class OpenGLIO
{
public:
    OpenGLIO();
    ~OpenGLIO();

    bool initialize();
    bool shutdown();
    bool send();
    bool receive(SpacecraftSim *scSim);
    void setIpAddress(std::string ipAddress);
    void setOutputSpacecraft(SpacecraftSim scSim);
    
private:
    SpacecraftSim scSim;
    TcpSerializeServer<SpacecraftSim, SpacecraftSim> server;
    std::string ipAddress;
};


class BoostCommunication
{
public:
    BoostCommunication();
    ~BoostCommunication();

    bool initializeAll(std::string tmpDataVariable);
    bool shutdownAll(std::string tmpDataVariable);

    OpenGLIO *getOpenGLIO() {
        return &this->openGLIO;
    }
   
private:
    OpenGLIO openGLIO;
    
};

#endif
