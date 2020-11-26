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
#ifndef MESSAGING2_H
#define MESSAGING2_H
#include <memory>
#include "_GeneralModuleFiles/sys_model.h"
#include <vector>
#include "architecture/messaging2/msg2Header.h"
#include "utilities/bskLogging.h"


/*! forward-declare sim message for use by read functor */
template<typename messageType>
class SimMessage;

template<typename messageType>
class Log;

/*! Read functors have read-only access to messages*/
template<typename messageType>
class ReadFunctor{
private:
    messageType* payloadPointer;    //! -- pointer to the incoming msg data
    Msg2Header *headerPointer;      //! -- pointer to the incoming msg header
    bool initialized;               //! -- flag indicating if the input message is connect to another message

public:
    BSKLogger bskLogger;                          //!< -- BSK Logging

    //! constructor
    ReadFunctor() : initialized(false) {};

    //! constructor
    ReadFunctor(messageType* payloadPtr, Msg2Header *headerPtr) : payloadPointer(payloadPtr), headerPointer(headerPtr), initialized(true){};

    //! constructor
    const messageType& operator()(){
        if (!this->initialized) {
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are trying to read an un-connected message.");
        }
        return *this->payloadPointer;

    };

    //! return a zero'd copy of the message payload structure
    messageType zeroMsgPayload(){
        messageType zeroMsg;
        memset(&zeroMsg, 0x0, sizeof(messageType));
        return zeroMsg;
    };

    //! copy the message payload from source to destination
    void copyMsgPayload(messageType *destination, messageType *source){
        memcpy(destination, source, sizeof(messageType));
        return;
    };


    //! check if this msg has been connected to
    bool isLinked(){return this->initialized;};  // something that can be checked so that uninitialized messages aren't read.

    //! check if the message has been ever written to
    bool isWritten(){
        if (this->initialized) {
            return this->headerPointer->isWritten;
        } else {
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are checking if an unconnected msg is written.");
            return false;
        }
    };

    //! return the time at which the message was written
    uint64_t timeWritten(){
        if (!this->initialized) {
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting the write time of an unconnected msg.");
            return 0;
        }
        return this->headerPointer->timeWritten;
    };

    //! return the moduleID of who wrote wrote the message
    int64_t moduleID(){
        if (!this->initialized) {
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting moduleID of an unconnected msg.");
            return 0;
        }
        if (!this->headerPointer->timeWritten) {
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting moduleID of an unwritten msg.");
            return 0;
        }
        return this->headerPointer->moduleID;
    };

    //! subscribe to a C message
    void subscribeToC(void* source){
        // this method works by knowing that the first member of a C message is the payload.
        this->payloadPointer = (messageType*) source;

        // advance the address to connect to C message header
        messageType* pt = this->payloadPointer;
        messageType** pt2 = (messageType **) (++pt);
        this->headerPointer = (Msg2Header *) (++pt2);

        // set flag that this input message is connected to another message
        this->initialized = true;           // set input message as linked
        this->headerPointer->isLinked = 1;  // set source output message as linked
    };

    //! Subscribe to a C++ message
    void subscribeTo(SimMessage<messageType> source){
        *this = source.addSubscriber();
        this->initialized = true;
    };

    //! Log method description
    Log<messageType> log(){return Log<messageType>(this);}
};

/*! Write Functor */
template<typename messageType>
class WriteFunctor{
private:
    messageType* payloadPointer;
    Msg2Header* headerPointer;
public:
    //! method description
    WriteFunctor(){};
    //! method description
    WriteFunctor(messageType* payloadPointer, Msg2Header *headerPointer) : payloadPointer(payloadPointer), headerPointer(headerPointer){};
    //! method description
    void operator()(messageType *payload, int64_t moduleID, uint64_t callTime){
        *this->payloadPointer = *payload;
        this->headerPointer->isWritten = 1;
        this->headerPointer->timeWritten = callTime;
        this->headerPointer->moduleID = moduleID;
        return;
    }
};

template<typename messageType>
class Log;

/*!
 * base class template for bsk messages
 */
template<typename messageType>
class SimMessage{
private:
    messageType payload = {};   //! struct defining message payload, zero'd on creation
    Msg2Header header = {};     //! struct defining the message header, zero'd on creation
    ReadFunctor<messageType> read = ReadFunctor<messageType>(&payload, &header);
    WriteFunctor<messageType> write = WriteFunctor<messageType>(&payload, &header);
public:
    //! -- request read rights. returns ref to this->read
    ReadFunctor<messageType> addSubscriber();
    //! -- request write rights.
    WriteFunctor<messageType> addAuthor();
    //! for plain ole c modules
    messageType* subscribeRaw(Msg2Header **msgPtr);
    Log<messageType> log(){return Log<messageType>(this);}
    //! - returned a zero'd copy of the messagy payload structure
    messageType zeroMsgPayload();
    //! - copies the source structure to the destination structure
    void copyMsgPayload(messageType *destination, messageType *source);

    //! check if this msg has been connected to
    bool isLinked(){return this->header.isLinked;};
};

template<typename messageType>
messageType SimMessage<messageType>::zeroMsgPayload(){
    messageType zeroMsg;
    memset(&zeroMsg, 0x0, sizeof(messageType));
    return zeroMsg;
}

template<typename messageType>
void SimMessage<messageType>::copyMsgPayload(messageType *destination, messageType *source){
    memcpy(destination, source, sizeof(messageType));
    return;
}


template<typename messageType>
ReadFunctor<messageType> SimMessage<messageType>::addSubscriber(){
    this->header.isLinked = 1;
    return this->read;
}

template<typename messageType>
WriteFunctor<messageType> SimMessage<messageType>::addAuthor(){
    return this->write;
}

template<typename messageType>
messageType* SimMessage<messageType>::subscribeRaw(Msg2Header **msgPtr){
    *msgPtr = &this->header;
    this->header.isLinked = 1;
    return &this->payload;
}

/*! Keep a time history of messages accessible to users from python */
template<typename messageType>
class Log : public SysModel{
public:
    Log(){};
    //! -- Use this to log cpp messages
    Log(SimMessage<messageType>* message){
        this->readMessage = message->addSubscriber();
    }
    //! -- Use this to log C messages
    Log(void* message){
        Msg2Header msgHeader;
        this->readMessage = ReadFunctor<messageType>((messageType*) message, &msgHeader);
    }
    //! -- Use this to keep track of what someone is reading
    Log(ReadFunctor<messageType>* messageReader){
        this->readMessage = *messageReader;
    }
    ~Log(){};

    //! -- self initialization
    void SelfInit(){};
    //! -- cross initialization
    void CrossInit(){};
    //! Method description
    void IntegratedInit(){};
    //! -- Read and record the message at the owning task's rate
    void UpdateState(uint64_t CurrentSimNanos){
        this->logTimes.push_back(CurrentSimNanos);
        this->log.push_back(this->readMessage());
    };
    //! Reset method
    void Reset(uint64_t CurrentSimNanos){this->log.clear(); this->logTimes.clear();};  //!< -- Can only reset to 0 for now
    //! time method
    std::vector<uint64_t>& times(){return this->logTimes;}
    //! record method
    std::vector<messageType>& record(){return this->log;};

private:
    std::vector<messageType> log;           //!< vector of log messages
    std::vector<uint64_t> logTimes;         //!< vector of log times

private:
    ReadFunctor<messageType> readMessage;   //!< method description
};

#endif
