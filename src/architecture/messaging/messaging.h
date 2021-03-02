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
#ifndef MESSAGING_H
#define MESSAGING_H
#include <memory>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <vector>
#include "architecture/messaging/msg2Header.h"
#include "architecture/utilities/bskLogging.h"
#include <typeinfo>

/*! forward-declare sim message for use by read functor */
template<typename messageType>
class Message;

template<typename messageType>
class Recorder;

/*! Read functors have read-only access to messages*/
template<typename messageType>
class ReadFunctor{
private:
    messageType* payloadPointer;    //!< -- pointer to the incoming msg data
    Msg2Header *headerPointer;      //!< -- pointer to the incoming msg header
    bool initialized;               //!< -- flag indicating if the input message is connect to another message
    
public:
    //!< -- BSK Logging
    BSKLogger bskLogger;            //!< -- bsk logging instance
    messageType zeroMsgPayload ={}; //!< -- zero'd copy of the message payload type


    //! constructor
    ReadFunctor() : initialized(false) {};

    //! constructor
    ReadFunctor(messageType* payloadPtr, Msg2Header *headerPtr) : payloadPointer(payloadPtr), headerPointer(headerPtr), initialized(true){};

    //! constructor
    const messageType& operator()(){
        if (!this->initialized) {
            messageType var;
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are trying to read an un-connected message of type %s\nThis program is about to self destruct.",  typeid(var).name());
        }
        return *this->payloadPointer;

    };

    //! check if this msg has been connected to
    bool isLinked(){return this->initialized;};  // something that can be checked so that uninitialized messages aren't read.

    //! check if the message has been ever written to
    bool isWritten(){
        if (this->initialized) {
            return this->headerPointer->isWritten;
        } else {
            messageType var;
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are checking if an unconnected msg of type %s is written.", typeid(var).name());
            return false;
        }
    };

    //! return the time at which the message was written
    uint64_t timeWritten(){
        if (!this->initialized) {
            messageType var;
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting the write time of an unconnected msg of type %s.", typeid(var).name());
            return 0;
        }
        return this->headerPointer->timeWritten;
    };

    //! return the moduleID of who wrote wrote the message
    int64_t moduleID(){
        if (!this->initialized) {
            messageType var;
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting moduleID of an unconnected msg of type %s.", typeid(var).name());
            return 0;
        }
        if (!this->headerPointer->isWritten) {
            messageType var;
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting moduleID of an unwritten msg of type %s.", typeid(var).name());
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
    void subscribeTo(Message<messageType> *source){
        *this = source->addSubscriber();
        this->initialized = true;
    };

    //! Recorder method description
    Recorder<messageType> recorder(uint64_t timeDiff = 0){return Recorder<messageType>(this, timeDiff);}
};

/*! Write Functor */
template<typename messageType>
class WriteFunctor{
private:
    messageType* payloadPointer;    //!< pointer to the message payload
    Msg2Header* headerPointer;      //!< pointer to the message header
public:
    //! write functor constructor
    WriteFunctor(){};
    //! write functor constructor
    WriteFunctor(messageType* payloadPointer, Msg2Header *headerPointer) : payloadPointer(payloadPointer), headerPointer(headerPointer){};
    //! write functor constructor
    void operator()(messageType *payload, int64_t moduleID, uint64_t callTime){
        *this->payloadPointer = *payload;
        this->headerPointer->isWritten = 1;
        this->headerPointer->timeWritten = callTime;
        this->headerPointer->moduleID = moduleID;
        return;
    }
};

template<typename messageType>
class Recorder;

/*!
 * base class template for bsk messages
 */
template<typename messageType>
class Message{
private:
    messageType payload = {};   //!< struct defining message payload, zero'd on creation
    Msg2Header header = {};     //!< struct defining the message header, zero'd on creation
    ReadFunctor<messageType> read = ReadFunctor<messageType>(&payload, &header);  //!< read functor instance
public:
    //! write functor to this message
    WriteFunctor<messageType> write = WriteFunctor<messageType>(&payload, &header);
    //! -- request read rights. returns reference to class ``read`` variable
    ReadFunctor<messageType> addSubscriber();
    //! -- request write rights.
    WriteFunctor<messageType> addAuthor();
    //! for plain ole c modules
    messageType* subscribeRaw(Msg2Header **msgPtr);
    //! Recorder object
    Recorder<messageType> recorder(uint64_t timeDiff = 0){return Recorder<messageType>(this, timeDiff);}
    
    messageType zeroMsgPayload = {};    //!< zero'd copy of the message payload structure

    //! check if this msg has been connected to
    bool isLinked(){return this->header.isLinked;};
};


template<typename messageType>
ReadFunctor<messageType> Message<messageType>::addSubscriber(){
    this->header.isLinked = 1;
    return this->read;
}

template<typename messageType>
WriteFunctor<messageType> Message<messageType>::addAuthor(){
    return this->write;
}

template<typename messageType>
messageType* Message<messageType>::subscribeRaw(Msg2Header **msgPtr){
    *msgPtr = &this->header;
    this->header.isLinked = 1;
    return &this->payload;
}

/*! Keep a time history of messages accessible to users from python */
template<typename messageType>
class Recorder : public SysModel{
public:
    Recorder(){};
    //! -- Use this to record cpp messages
    Recorder(Message<messageType>* message, uint64_t timeDiff = 0){
        this->timeInterval = timeDiff;
        this->readMessage = message->addSubscriber();
    }
    //! -- Use this to record C messages
    Recorder(void* message, uint64_t timeDiff = 0){
        this->timeInterval = timeDiff;
        Msg2Header msgHeader;
        this->readMessage = ReadFunctor<messageType>((messageType*) message, &msgHeader);
    }
    //! -- Use this to keep track of what someone is reading
    Recorder(ReadFunctor<messageType>* messageReader, uint64_t timeDiff = 0){
        this->timeInterval = timeDiff;
        this->readMessage = *messageReader;
        if (!messageReader->isLinked()) {
            messageType var;
            bskLogger.bskLog(BSK_ERROR, "In C++ read functor, you are requesting to record an un-connected input message of type %s.", typeid(var).name());
        }
    }
    ~Recorder(){};

    //! -- self initialization
    void SelfInit(){};
    //! -- cross initialization
    void IntegratedInit(){};
    //! -- Read and record the message
    void UpdateState(uint64_t CurrentSimNanos){
        if (CurrentSimNanos >= this->nextUpdateTime) {
            this->msgRecordTimes.push_back(CurrentSimNanos);
            this->msgWrittenTimes.push_back(this->readMessage.timeWritten());
            this->msgRecord.push_back(this->readMessage());
            this->nextUpdateTime += this->timeInterval;
        }
    };
    //! Reset method
    void Reset(uint64_t CurrentSimNanos){
        this->msgRecord.clear();    //!< -- Can only reset to 0 for now
        this->msgRecordTimes.clear();
        this->msgWrittenTimes.clear();
        this->nextUpdateTime = CurrentSimNanos;
    };
    //! time recorded method
    std::vector<uint64_t>& times(){return this->msgRecordTimes;}
    //! time written method
    std::vector<uint64_t>& timesWritten(){return this->msgWrittenTimes;}
    //! record method
    std::vector<messageType>& record(){return this->msgRecord;};

    //! clear the recorded messages, i.e. purge the history
    void clear(){
        this->msgRecord.clear();
        this->msgRecordTimes.clear();
        this->msgWrittenTimes.clear();
    };

    BSKLogger bskLogger;                          //!< -- BSK Logging

    //! method to update the minimum time interval before recording the next message
    void updateTimeInterval(uint64_t timeDiff) {
        this->timeInterval = timeDiff;
    };

private:
    std::vector<messageType> msgRecord;           //!< vector of recorded messages
    std::vector<uint64_t> msgRecordTimes;         //!< vector of times at which messages are recorded
    std::vector<uint64_t> msgWrittenTimes;        //!< vector of times at which messages are written
    uint64_t nextUpdateTime = 0;                  //!< [ns] earliest time at which the msg is recorded again
    uint64_t timeInterval;                        //!< [ns] recording time intervale

private:
    ReadFunctor<messageType> readMessage;   //!< method description
};

#endif
