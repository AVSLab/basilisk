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
    return;
}

MessageRouter::~MessageRouter()
{
    return;
}

void MessageRouter::initializeServer(std::string hostName, uint32_t portStart)
{
    serverConnection = new TcpServer(&ioService);
    serverConnection->acceptConnections(hostName, portStart);
}
