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
#include "architecture/asio/message_router/message_router.h"
#include <iostream>
#include "utilities/bsk_Print.h"

MessageRouter::MessageRouter(BasicIoObject_t<boost::asio::ip::tcp::socket> *inConnection)
{
    serverConnection = nullptr;
    clientConnection = nullptr;
    theConnection = inConnection;
    runAsServer = false;
    processLinked = false;
    hostName = "localhost";
    defaultPort = 20000;
    this->blockConnection = true;
    return;
}

MessageRouter::~MessageRouter()
{
    
    theConnection->close();
    
    if(serverConnection)
    {
        delete serverConnection;
    }
    if(clientConnection)
    {
        delete clientConnection;
    }
    return;
}

MessageRouter::MessageRouter(std::string from, std::string to, std::string intName, BasicIoObject_t<boost::asio::ip::tcp::socket> *inConnection) : MessageRouter()
{
    processData.messageSource = from;
    processData.messageDest = to;
    if(intName == "")
    {
        intName = from + "2" + to + "Interface";
    }
    ModelTag = intName;
    theConnection = inConnection;
}

bool MessageRouter::initializeServer(std::string hostName, uint32_t portStart)
{
    bool serverLinked;
    serverConnection = new TcpServer(&ioService);
    if(theConnection != nullptr)
    {
        serverConnection->m_stream = theConnection->m_stream;
        serverLinked = serverConnection->isOpen();
    }
    else
    {
        serverLinked = serverConnection->acceptConnections(hostName, portStart);
    }
	serverConnection->m_stream->set_option(boost::asio::ip::tcp::no_delay(false));
    theConnection = serverConnection;
    return(serverLinked);
}

bool MessageRouter::initializeClient(std::string hostName, uint32_t portStart)
{
    bool clientLinked = false;
    clientConnection = new TcpClient(&ioService);
    if(theConnection != nullptr)
    {
        clientConnection->m_stream = theConnection->m_stream;
        clientLinked = clientConnection->isOpen();
    }
    else
    {
        clientLinked = !(clientConnection->connect(hostName, portStart));
    }
	clientConnection->m_stream->set_option(boost::asio::ip::tcp::no_delay(false));
    theConnection = clientConnection;
    return(clientLinked);
}

bool MessageRouter::linkProcesses()
{
    
    if(processLinked)
    {
        return processLinked;
    }
    std::string outputString = processData.messageDest;
    processData.source = SystemMessaging::GetInstance()->
    findMessageBuffer(processData.messageSource);
    if(runAsServer)
    {
        processLinked = initializeServer(hostName, defaultPort);
        uint32_t stringLength = (uint32_t)processData.messageDest.length();
        std::vector<char> outData((char *) (&stringLength), (char *) &stringLength+sizeof(stringLength));
        theConnection->sendData(outData);
        outData.clear();
        outData.insert(outData.begin(), processData.messageDest.c_str(), processData.messageDest.c_str() + stringLength);
        outData.push_back(0);
        theConnection->sendData(outData);
        processLinked = true;
    }
    else
    {
        processLinked = initializeClient(hostName, defaultPort);
        std::vector<char> inData(4, 0xFF);
        theConnection->receiveData(inData);
        uint32_t stringLength;
        memcpy(&stringLength, inData.data(), sizeof(stringLength));
        inData.clear();
        inData.insert(inData.begin(), stringLength+1, 0xFF);
        theConnection->receiveData(inData);
        processData.messageDest = inData.data();
        processData.destination = SystemMessaging::GetInstance()->
            findMessageBuffer(processData.messageDest);
        processLinked = true;
        if(processData.destination < 0)
        {
            BSK_PRINT_BRIEF(MSG_ERROR, "Failed to find a messaging buffer with name: %s", processData.messageDest.c_str());
            processLinked = false;
        }
    }
    return processLinked;
}

void MessageRouter::requestUnknownMessages()
{
    std::set<std::string> unknownPublisher;
    processData.destination = SystemMessaging::GetInstance()->
    findMessageBuffer(processData.messageDest);
    SystemMessaging::GetInstance()->selectMessageBuffer(processData.destination);
    unknownPublisher = SystemMessaging::GetInstance()->getUnpublishedMessages();
    std::set<std::string>::iterator it;
    MessageNameIDPair *messageBuffer = new MessageNameIDPair[unknownPublisher.size()];
    memset(messageBuffer, 0x0, unknownPublisher.size()*sizeof(MessageNameIDPair));
    int messCount = 0;
    for(it=unknownPublisher.begin(); it != unknownPublisher.end(); it++)
    {
        strcpy(messageBuffer[messCount].messageName, it->c_str());
        messageBuffer[messCount].messageID =
            SystemMessaging::GetInstance()->FindMessageID(*it);
        SystemMessaging::GetInstance()->
            obtainWriteRights(messageBuffer[messCount].messageID, moduleID);
        messCount++;
    }
    uint32_t messageLength = (uint32_t)unknownPublisher.size()*sizeof(MessageNameIDPair);
    std::vector<char> outData((char *) (&messageLength),
                              (char *) &messageLength+sizeof(messageLength));
    theConnection->sendData(outData);
    outData.clear();
    outData.insert(outData.begin(), (char*) messageBuffer, (char*) messageBuffer + messageLength);
    theConnection->sendData(outData);
    
    
}

void MessageRouter::receiveUnknownMessages()
{
    std::vector<char> inData(4, 0xFF);
    theConnection->receiveData(inData);
    uint32_t stringLength;
    uint32_t messageCount;
    memcpy(&stringLength, inData.data(), sizeof(stringLength));
    messageCount = stringLength/sizeof(MessageNameIDPair);
    inData.clear();
    inData.insert(inData.begin(), stringLength, 0xFF);
    theConnection->receiveData(inData);
    MessageNameIDPair *messageBuffer = new MessageNameIDPair[messageCount];
    memset(messageBuffer, 0x0, messageCount*sizeof(MessageNameIDPair));
    memcpy(messageBuffer, inData.data(), stringLength);
    processData.source = SystemMessaging::GetInstance()->
    findMessageBuffer(processData.messageSource);
    SystemMessaging::GetInstance()->selectMessageBuffer(processData.source);
    for(int i=0; i<messageCount; i++)
    {
        int64_t messID = SystemMessaging::GetInstance()->
            FindMessageID(messageBuffer[i].messageName);
        MessageWriteData writeInfo;
        if(messID >= 0)
        {
            writeInfo.messageID = messageBuffer[i].messageID;
            writeInfo.lastWriteCounter = 0;
            routerMapping.insert(std::pair<int64_t, MessageWriteData>
                (messID, writeInfo));
        }
    }
}

void MessageRouter::discoverMessages()
{
    if(runAsServer)
    {
        receiveUnknownMessages();
    }
    else
    {
        requestUnknownMessages();
    }
}

void MessageRouter::UpdateState(uint64_t CurrentSimNanos)
{
    if(!runAsServer  || !theConnection->isOpen())
    {
        return;
    }
    
    MessageHeaderData* msgHdr;
    std::map<int64_t, MessageWriteData>::iterator it;
    std::vector<char> outSize;
    std::vector<char> outPayload;
    ExchangeMessageHeader localHdr;
    SystemMessaging::GetInstance()->selectMessageBuffer(processData.source);
    for(it=routerMapping.begin(); it != routerMapping.end(); it++)
    {
        msgHdr = SystemMessaging::GetInstance()->FindMsgHeader(it->first);
        memset(&localHdr, 0x0, sizeof(SingleMessageHeader));
        if(msgHdr->UpdateCounter != it->second.lastWriteCounter)
        {
            uint8_t *msgBuffer = new uint8_t[msgHdr->CurrentReadSize];
            memset(msgBuffer, 0x0, msgHdr->CurrentReadSize);
            SystemMessaging::GetInstance()->
                ReadMessage(it->first, &(localHdr.headerData),
                msgHdr->CurrentReadSize, msgBuffer);
            localHdr.messageID = it->second.messageID;
            outPayload.insert(outPayload.end(), (char*) &localHdr,
                (char*) &localHdr + sizeof(localHdr));
            outPayload.insert(outPayload.end(), (char*) msgBuffer,
                (char*) msgBuffer + localHdr.headerData.WriteSize);
            it->second.lastWriteCounter = msgHdr->UpdateCounter;
            delete [] msgBuffer;
        }
    }
    uint32_t bufferSize = (uint32_t)outPayload.size();
    outSize.insert(outSize.begin(), (char*) &bufferSize,
        (char*) &bufferSize + sizeof(bufferSize));
    theConnection->sendData(outSize);
    theConnection->sendData(outPayload);
}

void MessageRouter::routeMessages()
{
    if(!theConnection->isOpen() || (!blockConnection && theConnection->bytesWaiting() == 0 ))
    {
        return;
    }
    while(this->theConnection->bytesWaiting() > 0) {
    std::vector<char> inSize(4, 0x00);
    theConnection->receiveData(inSize);
    uint32_t stringLength;
    memcpy(&stringLength, inSize.data(), sizeof(stringLength));
    if(stringLength == 0)
    {
        continue;
    }
    std::vector<char> inData(stringLength, 0xFF);
    theConnection->receiveData(inData);
    uint64_t bytesRead = 0;
    ExchangeMessageHeader *localHdr;
    SystemMessaging::GetInstance()->selectMessageBuffer(processData.destination);
    while(bytesRead < inData.size())
    {
        localHdr = reinterpret_cast<ExchangeMessageHeader *>(&(inData.data()[bytesRead]));
        bytesRead += sizeof(ExchangeMessageHeader);
        SystemMessaging::GetInstance()->WriteMessage(localHdr->messageID,
            localHdr->headerData.WriteClockNanos, localHdr->headerData.WriteSize,
            (uint8_t *)(&(inData.data()[bytesRead])));
        bytesRead += localHdr->headerData.WriteSize;
    }
    }
    
}

void MessageRouter::closeConnection()
{
    theConnection->close();
}

