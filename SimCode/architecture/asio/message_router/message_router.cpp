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
#include "architecture/asio/message_router/message_router.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>

MessageRouter::MessageRouter()
{
    serverConnection = nullptr;
    clientConnection = nullptr;
    theConnection = nullptr;
    runAsServer = false;
    hostName = "localhost";
    defaultPort = 20000;
    return;
}

MessageRouter::~MessageRouter()
{
    return;
}

MessageRouter::MessageRouter(std::string from, std::string to, std::string intName)
{
    processData.messageSource = from;
    processData.messageDest = to;
    if(intName == "")
    {
        intName = from + "2" + to + "Interface";
    }
    ModelTag = intName;
}

bool MessageRouter::initializeServer(std::string hostName, uint32_t portStart)
{
    bool serverLinked;
    serverConnection = new TcpServer(&ioService);
    serverLinked = serverConnection->acceptConnections(hostName, portStart);
    theConnection = serverConnection;
    return(serverLinked);
}

bool MessageRouter::initializeClient(std::string hostName, uint32_t portStart)
{
    bool clientLinked;
    clientConnection = new TcpClient(&ioService);
    clientLinked = clientConnection->connect(hostName, portStart);
    theConnection = clientConnection;
    return(clientLinked);
}

bool MessageRouter::linkProcesses()
{
    bool processLinked = false;
    processData.source = SystemMessaging::GetInstance()->
    findMessageBuffer(processData.messageSource);
    if(runAsServer)
    {
        processLinked = initializeServer(hostName, defaultPort);
        uint32_t stringLength = processData.messageDest.length();
        
        theConnection->sendData(processData.messageDest);
    }
    else
    {
        processLinked = initializeClient(hostName, defaultPort);
        
    }
    return processLinked;
}

void MessageRouter::discoverMessages()
{
    
}

void MessageRouter::routeUnknownMessages()
{
    
}