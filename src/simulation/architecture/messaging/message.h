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
#ifndef MESSAGE_H
#define MESSAGE_H
#include <memory>
#include "../../_GeneralModuleFiles/sys_model.h"
#include <vector>

/*! \addtogroup SimArchGroup
 * @{
 */
/*! forward-declare sim message for use by read functor */
template<typename messageType>
class SimMessage;

template<typename messageType>
class Log;

/*! Read functors have read-only access to messages*/
template<typename messageType>
class ReadFunctor{
private:
    messageType* payloadPointer;
    bool initialized;
public:
    ReadFunctor() : initialized(false) {};
    ReadFunctor(messageType* payloadPointer) : payloadPointer(payloadPointer), initialized(true){};
    const messageType& operator()(){return *this->payloadPointer;};
    bool linked(){return this->initialized;};  // something that can be checked so that uninitialized messages aren't read.
    void subscribeToC(void* source){
        this->payloadPointer = (messageType*) source;
        this->initialized = true;} // this method works by knowing that the first member of a C message is the payload.
    void subscribeTo(SimMessage<messageType> source){
        *this = source.addSubscriber();
    }
    Log<messageType> log(){return Log<messageType>(this);}
};

template<typename messageType>
class WriteFunctor{
private:
    messageType* payloadPointer;
public:
    WriteFunctor(){};
    WriteFunctor(messageType* payloadPointer) : payloadPointer(payloadPointer){};
    void operator()(messageType payload){
        *this->payloadPointer = payload;
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
    messageType payload;  //! -- struct defining message payload
    ReadFunctor<messageType> read = ReadFunctor<messageType>(&payload);
    WriteFunctor<messageType> write = WriteFunctor<messageType>(&payload);
public:
    ReadFunctor<messageType> addSubscriber();  //! -- request read rights. returns ref to this->read
    WriteFunctor<messageType> addAuthor();  //! -- request write rights.
    messageType* subscribeRaw();  //! for plain ole c modules
    Log<messageType> log(){return Log<messageType>(this);}
};

template<typename messageType>
ReadFunctor<messageType> SimMessage<messageType>::addSubscriber(){
    return this->read;
}

template<typename messageType>
WriteFunctor<messageType> SimMessage<messageType>::addAuthor(){
    return this->write;
}

template<typename messageType>
messageType* SimMessage<messageType>::subscribeRaw(){
    return &this->payload;
}

/*! Keep a time history of messages accessible to users from python */
template<typename messageType>
class Log : public SysModel{
public:
    Log(){};
    Log(SimMessage<messageType>* message){
        this->readMessage = message->addSubscriber();
    }  //!< -- Use this to log cpp messages
    Log(void* message){
        this->readMessage = ReadFunctor<messageType>((messageType*) message);
    } //!< -- Use this to log C messages
    Log(ReadFunctor<messageType>* messageReader){
        this->readMessage = *messageReader;
    }  //!< -- Use this to keep track of what someone is reading
    ~Log(){};
    void SelfInit(){};
    void CrossInit(){};
    void IntegratedInit(){};
    void UpdateState(uint64_t CurrentSimNanos){
        this->logTimes.push_back(CurrentSimNanos);
        this->log.push_back(this->readMessage());
    };  //!< -- Read and record the message at the owning task's rate
    void Reset(uint64_t CurrentSimNanos){this->log.clear(); this->logTimes.clear();};  //!< -- Can only reset to 0 for now
    std::vector<uint64_t>& times(){return this->logTimes;}
    std::vector<messageType>& record(){return this->log;};

private:
    std::vector<messageType> log;
    std::vector<uint64_t> logTimes;

private:
    ReadFunctor<messageType> readMessage;
};

#endif
