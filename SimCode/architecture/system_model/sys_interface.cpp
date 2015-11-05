
#include "architecture/system_model/sys_interface.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>

InterfaceDataExchange::InterfaceDataExchange()
{
    exchangeActive = true;
    messageTraffic.clear();
    processData.messageDest = "";
    processData.messageSource = "";
    processData.destination = -1;
    processData.source = -1;
    msgBufferSize = 0;
    msgBuffer = NULL;
}

InterfaceDataExchange::~InterfaceDataExchange()
{

}

bool InterfaceDataExchange::linkProcesses()
{
    bool buffersFound = true;
    processData.destination = SystemMessaging::GetInstance()->
    findMessageBuffer(processData.messageDest);
    if(processData.destination < 0)
    {
        std::cerr << "Failed to find a messaging buffer with name: ";
        std::cerr << processData.messageDest << std::endl;
        buffersFound = false;
    }
    processData.source = SystemMessaging::GetInstance()->
    findMessageBuffer(processData.messageSource);
    if(processData.source < 0)
    {
        std::cerr << "Failed to find a messaging buffer with name: ";
        std::cerr << processData.messageSource << std::endl;
        buffersFound = false;
    }
    return(buffersFound);
}

void InterfaceDataExchange::discoverMessages()
{
    std::set<std::string> unknownPublisher;
    std::set<std::string>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(processData.destination);
    unknownPublisher = SystemMessaging::GetInstance()->getUnpublishedMessages();
    SystemMessaging::GetInstance()->selectMessageBuffer(processData.source);
    for(it=unknownPublisher.begin(); it!=unknownPublisher.end(); it++)
    {
        int64_t messageID = SystemMessaging::GetInstance()->FindMessageID(*it);
        if(messageID >= 0)
        {
            MessageInterfaceMatch newMessage;
            newMessage.source = -1;
            newMessage.destination = -1;
            newMessage.messageSource = *it;
            newMessage.messageDest = "";
            newMessage.updateCounter = 0;
            messageTraffic.push_back(newMessage);
        }
    }
    
}

bool InterfaceDataExchange::linkMessages()
{
    bool messagesLinked = true;
    std::vector<MessageInterfaceMatch>::iterator it;
    for(it=messageTraffic.begin(); it != messageTraffic.end(); it++)
    {
        SystemMessaging::GetInstance()->
            selectMessageBuffer(processData.destination);
        it->destination = SystemMessaging::GetInstance()->
            FindMessageID(it->messageSource);
        SystemMessaging::GetInstance()->
        selectMessageBuffer(processData.source);
        it->source = SystemMessaging::GetInstance()->
        FindMessageID(it->messageSource);
        if(it->destination < 0 || it->source < 0)
        {
            messagesLinked = false;
        }
    }
    return(messagesLinked);
}

void InterfaceDataExchange::routeMessages()
{
    SingleMessageHeader dataHeader;
    std::vector<MessageInterfaceMatch>::iterator it;
    for(it=messageTraffic.begin(); it != messageTraffic.end(); it++)
    {
        SystemMessaging::GetInstance()->
        selectMessageBuffer(processData.source);
        MessageHeaderData* localHdr = SystemMessaging::GetInstance()->
            FindMsgHeader(it->source);
        if(localHdr->MaxMessageSize > msgBufferSize)
        {
            if(msgBuffer != NULL)
            {
                delete [] msgBuffer;
            }
            msgBuffer = new uint8_t[localHdr->MaxMessageSize];
            memset(msgBuffer, 0x0, localHdr->MaxMessageSize);
            msgBufferSize = localHdr->MaxMessageSize;
        }
        if(localHdr->UpdateCounter == it->updateCounter)
        {
            continue;
        }
        SystemMessaging::GetInstance()->ReadMessage(it->source, &dataHeader,
            localHdr->MaxMessageSize, msgBuffer);
        SystemMessaging::GetInstance()->
        selectMessageBuffer(processData.destination);
        SystemMessaging::GetInstance()->WriteMessage(it->destination,
            dataHeader.WriteClockNanos, dataHeader.WriteSize, msgBuffer);
        it->updateCounter = localHdr->UpdateCounter;
        
    }
}

SysInterface::SysInterface()
{
    interfaceActive = true;
    interfacesLinked = false;
}

SysInterface::~SysInterface()
{
    std::vector<InterfaceDataExchange*>::iterator itPtr;
    for(itPtr = interfaceDef.begin(); itPtr != interfaceDef.end(); itPtr++)
    {
        delete (*itPtr);
    }
}

void SysInterface::addNewInterface(InterfaceDataExchange * newInterface)
{
    interfaceDef.push_back(newInterface);
    std::vector<MessageInterfaceMatch>::iterator it;
    newInterface->exchangeActive = true;
    newInterface->processData.messageDest = -1;
    newInterface->processData.messageSource = -1;
    for(it = newInterface->messageTraffic.begin();
        it != newInterface->messageTraffic.end(); it++)
    {
        it->destination = -1;
        it->source = -1;
    }
    interfacesLinked = false;
}
void SysInterface::addNewInterface(std::string from, std::string to)
{

    InterfaceDataExchange *newInterface = new InterfaceDataExchange();
    std::vector<InterfaceDataExchange*>::iterator it;
    newInterface->processData.messageSource = from;
    newInterface->processData.messageDest = to;
    interfaceDef.push_back(newInterface);
    interfacesLinked = false;
    it = interfaceDef.end() - 1;
    currentInterface = (*it);

}
void SysInterface::connectInterfaces()
{
    std::vector<InterfaceDataExchange*>::iterator it;
    for(it=interfaceDef.begin(); it!=interfaceDef.end(); it++)
    {
        std::vector<MessageInterfaceMatch>::iterator messIt;
        if(!((*it)->linkProcesses()))
        {
            std::cerr << "Interface failed to link.  Disabling." << std::endl;
            (*it)->exchangeActive = false;
            continue;
        }
        (*it)->linkMessages();
    }
    interfacesLinked = true;
}
void SysInterface::routeInputs(uint64_t processBuffer)
{
    std::vector<InterfaceDataExchange*>::iterator it;
    
    if(!interfaceActive)
    {
        return;
    }
    if(!interfacesLinked)
    {
        connectInterfaces();
    }

    for(it=interfaceDef.begin(); it!=interfaceDef.end(); it++)
    {
        (*it)->routeMessages();
    }
    

}
void SysInterface::discoverAllMessages()
{
    std::vector<InterfaceDataExchange*>::iterator it;
    for(it=interfaceDef.begin(); it!=interfaceDef.end(); it++)
    {
        (*it)->linkProcesses();
        (*it)->discoverMessages();
    }
}

