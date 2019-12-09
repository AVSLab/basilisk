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

#include "_GeneralModuleFiles/sys_interface.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>

/*!
 * Contruct an InterfaceDataExchange()
 */
InterfaceDataExchange::InterfaceDataExchange()
{
    this->exchangeActive = true;
    this->messageTraffic.clear();
    this->processData.messageDest = "";
    this->processData.messageSource = "";
    this->processData.destination = -1;
    this->processData.source = -1;
    this->msgBufferSize = 0;
    this->msgBuffer = NULL;
    this->needDelete = false;
}

/*!
 * Destroy an InterfaceDataExchange()
 */
InterfaceDataExchange::~InterfaceDataExchange()
{
}

/*!
 * Get IDs for the to/from processes
 * @return bool buffersFound
 */
bool InterfaceDataExchange::linkProcesses()
{
    bool buffersFound = true;
    this->processData.destination = SystemMessaging::GetInstance()->
    findMessageBuffer(this->processData.messageDest);
    if(this->processData.destination < 0)
    {
        bskLogger.bskLog(ERROR, "Failed to find a messaging buffer with name: "
                                   "%s", this->processData.messageDest.c_str());
        buffersFound = false;
    }
    this->processData.source = SystemMessaging::GetInstance()->
    findMessageBuffer(this->processData.messageSource);
    if(this->processData.source < 0)
    {
        bskLogger.bskLog(ERROR, "Failed to find a messaging buffer with name: %s", this->processData.messageSource.c_str());
        buffersFound = false;
    }
    return(buffersFound);
}

/*!
 * This method checks the destination buffer for unpublished messages
 * Then, it switches to the source buffer and if the message exists, adds it to the messageTraffic
 */
void InterfaceDataExchange::discoverMessages()
{
    std::set<std::string> unknownPublisher;
    std::set<std::string>::iterator it;
    SystemMessaging::GetInstance()->selectMessageBuffer(this->processData.destination);
    unknownPublisher = SystemMessaging::GetInstance()->getUnpublishedMessages();
    SystemMessaging::GetInstance()->selectMessageBuffer(this->processData.source);
    for(it=unknownPublisher.begin(); it!=unknownPublisher.end(); it++)
    {
        int64_t messageID = SystemMessaging::GetInstance()->FindMessageID(*it);
        if(messageID >= 0)
        {
            MessageInterfaceMatch newMessage;
            newMessage.source = -1;  // Message ID in the source buffer
            newMessage.destination = -1;  // ID for the message within the destination buffer
            newMessage.messageSource = *it; // The name of the message that was published elsewhere
            newMessage.messageDest = "";
            newMessage.updateCounter = 0;
            this->messageTraffic.push_back(newMessage);
        }
    }
}

/*!
 * This method links the messages across buffers, which means:
 * 1) It gets Write permission in the destination buffer
 * 2) It gets Read permision in the source buffer
 * @return bool messagedLinked whether or not the messages are linked
 */
bool InterfaceDataExchange::linkMessages()
{
    bool messagesLinked = true;
    std::vector<MessageInterfaceMatch>::iterator it;
    for(it=this->messageTraffic.begin(); it != this->messageTraffic.end(); it++)
    {
        SystemMessaging::GetInstance()->
            selectMessageBuffer(this->processData.destination);
        it->destination = SystemMessaging::GetInstance()->
            FindMessageID(it->messageSource);
        if(it->destination >= 0)
        {
            SystemMessaging::GetInstance()->obtainWriteRights(it->destination,
                                                              this->moduleID);
        }
        SystemMessaging::GetInstance()->
        selectMessageBuffer(this->processData.source);
        it->source = SystemMessaging::GetInstance()->
        FindMessageID(it->messageSource);
        if(it->source >= 0)
        {
            SystemMessaging::GetInstance()->obtainReadRights(it->source,
                                                             this->moduleID);
        }
        if(it->destination < 0 || it->source < 0)
        {
            messagesLinked = false;
        }
    }
    return(messagesLinked);
}

/*!
 * Read all messages from source and write to destination
 * @return void
 */
void InterfaceDataExchange::routeMessages()
{
    SingleMessageHeader dataHeader;
    std::vector<MessageInterfaceMatch>::iterator it;
    for(it=this->messageTraffic.begin(); it!=this->messageTraffic.end(); it++)
    {
        SystemMessaging::GetInstance()->
        selectMessageBuffer(processData.source);
        MessageHeaderData* localHdr = SystemMessaging::GetInstance()->
            FindMsgHeader(it->source);
        if(localHdr->MaxMessageSize > this->msgBufferSize)
        {
            if(this->msgBuffer != NULL)
            {
                delete [] this->msgBuffer;
            }
            this->msgBuffer = new uint8_t[localHdr->MaxMessageSize];
            memset(this->msgBuffer, 0x0, localHdr->MaxMessageSize);
            this->msgBufferSize = localHdr->MaxMessageSize;
        }
        // Don't route it if it hasn't been updated
        if(localHdr->UpdateCounter == it->updateCounter)
        {
            continue;
        }
        SystemMessaging::GetInstance()->ReadMessage(it->source, &dataHeader,
            localHdr->MaxMessageSize, msgBuffer, moduleID);
        SystemMessaging::GetInstance()->
        selectMessageBuffer(this->processData.destination);
        SystemMessaging::GetInstance()->WriteMessage(it->destination,
            dataHeader.WriteClockNanos, dataHeader.WriteSize, this->msgBuffer, this->moduleID);
        it->updateCounter = localHdr->UpdateCounter;
    }
}

/*!
 * Create a SysInterface
 */
SysInterface::SysInterface()
{
    this->interfaceActive = true;
    this->interfacesLinked = false;
}

/*!
 * Destruct a SysInterface
 */
SysInterface::~SysInterface()
{
    std::vector<InterfaceDataExchange*>::iterator itPtr;
    for(itPtr = this->interfaceDef.begin(); itPtr != this->interfaceDef.end(); itPtr++)
    {
        if((*itPtr)->needDelete)
        {
            delete (*itPtr);
        }
    }
}

/*!
 * Add an existing interface, but resets the destination and source to -1
 * @param InterfaceDataExchange* newInterface
 * @return void
 */
void SysInterface::addNewInterface(InterfaceDataExchange * newInterface)
{
    this->interfaceDef.push_back(newInterface);
    std::vector<MessageInterfaceMatch>::iterator it;
    newInterface->exchangeActive = true;
    newInterface->processData.destination = -1;
    newInterface->processData.source = -1;
    for(it = newInterface->messageTraffic.begin();
        it != newInterface->messageTraffic.end(); it++)
    {
        it->destination = -1;
        it->source = -1;
    }
    this->interfacesLinked = false;
}

/*!
 * Create a new interface from "from" process to "to" process.
 * Auto-names the interface if the name provided is "".
 * @param std::string from process to read messages from
 * @param std::string to process to copy messages to
 * @param std::string intName name of this new interface
 * @return void
 */
void SysInterface::addNewInterface(std::string from, std::string to, std::string intName)
{

    InterfaceDataExchange *newInterface = new InterfaceDataExchange();
    std::vector<InterfaceDataExchange*>::iterator it;
    newInterface->processData.messageSource = from;
    newInterface->processData.messageDest = to;
    if(intName == "")
    {
        intName = from + "2" + to + "Interface";
    }
    newInterface->ModelTag = intName;
    newInterface->needDelete = true;
    this->interfaceDef.push_back(newInterface);
    this->interfacesLinked = false;
    it = this->interfaceDef.end() - 1;
    this->currentInterface = (*it);
}

/*!
 * This method connects interfaces. If the processes won't link, the interface is disabled
 * and the user is warned
 * @return void
 */
void SysInterface::connectInterfaces()
{
    std::vector<InterfaceDataExchange*>::iterator it;
    for(it=interfaceDef.begin(); it!=interfaceDef.end(); it++)
    {
        std::vector<MessageInterfaceMatch>::iterator messIt;
        if(!((*it)->linkProcesses()))
        {
            bskLogger.bskLog(ERROR, "Interface failed to link.  Disabling.");
            (*it)->exchangeActive = false;
            continue;
        }
        (*it)->linkMessages();
    }
    this->interfacesLinked = true;
}

/*!
 * Routes messages. Only if the interface is active. If interfaces are not
 * linked, then they are connected first.
 * @param int64_t processBuffer not used
 * @return void
 */
void SysInterface::routeInputs(int64_t processBuffer)
{
    std::vector<InterfaceDataExchange*>::iterator it;
    
    if(!this->interfaceActive)
    {
        return;
    }
    if(!this->interfacesLinked)
    {
        connectInterfaces();
    }

    for(it=this->interfaceDef.begin(); it!=this->interfaceDef.end(); it++)
    {
        (*it)->routeMessages();
    }
}

/*!
 * This method goes over all interfaces and links processes and discovers messages
 */
void SysInterface::discoverAllMessages()
{
    std::vector<InterfaceDataExchange*>::iterator it;
    for(it=this->interfaceDef.begin(); it!=this->interfaceDef.end(); it++)
    {
        (*it)->linkProcesses();
        (*it)->discoverMessages();
    }
}

