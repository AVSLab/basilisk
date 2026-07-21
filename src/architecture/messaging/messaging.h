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
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <deque>
#include "architecture/messaging/msgHeader.h"
#include "architecture/messaging/messagingBase.h"
#include "architecture/utilities/bskLogging.h"
#include <typeinfo>
#include <stdlib.h>
#include <utility>
#include "architecture/messaging/payloadEqualityTraits.h"

/*! forward-declare sim message for use by read functor */
template<typename messageType>
class Message;

template<typename messageType>
class Recorder;

/*! Read functors have read-only access to messages*/
template<typename messageType>
class ReadFunctor : public ReadFunctorBase{
private:
    messageType* payloadPointer = nullptr; //!< -- pointer to the incoming msg data
    MsgHeader *headerPointer = nullptr;    //!< -- pointer to the incoming msg header
    bool initialized = false;              //!< -- flag indicating if the input message is connected to another message

    // --- issue #676 keep-alive bridge -------------------------------------------------------
    // When a Python-created stand-alone message is subscribed to, the SWIG layer installs an
    // opaque owner token (a PyObject*) plus acquire/release callbacks here so that the source
    // message cannot be garbage-collected while this reader still points into its memory.
    // These are plain C types so messaging.h stays free of <Python.h>: a no-Python build sees
    // null callbacks and pays nothing. The callbacks (set only from SWIG) do the GIL-safe
    // Py_INCREF/Py_DECREF; this header never touches Python.
    void* sourceHandle = nullptr;            //!< -- opaque owner token (a PyObject* in practice)
    void  (*acquireSource)(void*) = nullptr; //!< -- +1 the owner (Py_INCREF under the GIL)
    void  (*releaseSource)(void*) = nullptr; //!< -- -1 the owner (Py_DECREF under the GIL)
    bool replacingSource = false;            //!< -- prevents ownership changes during a release callback

    //! keep replacement callbacks from changing the reader until the incoming state is committed
    class SourceReplacementGuard {
    public:
        explicit SourceReplacementGuard(bool& replacementFlag)
            : flag(replacementFlag) {
            this->flag = true;
        }

        ~SourceReplacementGuard() {
            this->flag = false;
        }

    private:
        bool& flag;
    };

    //! own an acquired or transferred source reference until assignment commits it
    class SourceHandleGuard {
    public:
        SourceHandleGuard(void* handle,
                          void (*acquire)(void*),
                          void (*release)(void*),
                          bool acquireReference)
            : handle(handle),
              release(release) {
            if (acquireReference && this->handle && acquire) {
                acquire(this->handle);
            }
        }

        ~SourceHandleGuard() {
            if (this->handle && this->release) {
                this->release(this->handle);
            }
        }

        void relinquish() {
            this->handle = nullptr;
        }

    private:
        void* handle;
        void (*release)(void*);
    };

    //! release our hold on the current source (if any), then forget it
    void releaseHandle_() {
        void* handle = this->sourceHandle;
        void (*release)(void*) = this->releaseSource;
        this->sourceHandle = nullptr;
        this->acquireSource = nullptr;
        this->releaseSource = nullptr;
        if (handle && release) {
            release(handle);
        }
    }

    //! copy the keep-alive trio from another reader and take an additional reference to it
    void adoptHandleFrom_(const ReadFunctor& other) {
        this->sourceHandle = other.sourceHandle;
        this->acquireSource = other.acquireSource;
        this->releaseSource = other.releaseSource;
        if (this->sourceHandle && this->acquireSource) {
            this->acquireSource(this->sourceHandle);  // now two independent owners
        }
    }

public:
    //!< -- BSK Logging
    BSKLogger bskLogger;            //!< -- bsk logging instance
    messageType zeroMsgPayload ={}; //!< -- zero'd copy of the message payload type


    //! constructor
    ReadFunctor() = default;

    //! constructor
    ReadFunctor(messageType* payloadPtr, MsgHeader *headerPtr) :
                  payloadPointer(payloadPtr), headerPointer(headerPtr), initialized(true)
    {
        this->setPointerData(headerPtr, payloadPtr);
    };

    //! destructor -- REQUIRED for the #676 keep-alive: a ReadFunctor is usually a C++ member
    //! of a module, so SWIG %extend destructors never run for it; only this real C++
    //! destructor does, and it must drop the Python reference held via releaseSource.
    ~ReadFunctor() {
        SourceReplacementGuard replacementGuard(this->replacingSource);
        this->payloadPointer = nullptr;
        this->headerPointer = nullptr;
        this->clearPointerData();
        this->initialized = false;
        this->releaseHandle_();
    }

    //! copy constructor -- take an additional reference to the source
    ReadFunctor(const ReadFunctor& other)
        : payloadPointer(other.payloadPointer),
          headerPointer(other.headerPointer),
          initialized(other.initialized),
          bskLogger(other.bskLogger),
          zeroMsgPayload(other.zeroMsgPayload) {
        this->setPointerData(other.headerPointer, other.payloadPointer);
        this->adoptHandleFrom_(other);
    }

    //! copy assignment -- acquire the incoming owner before releasing and replacing ours
    ReadFunctor& operator=(const ReadFunctor& other) {
        if (this == &other) { return *this; }
        if (this->replacingSource) { return *this; }

        bool sameSource = (this->sourceHandle == other.sourceHandle);
        if (sameSource) {
            this->payloadPointer = other.payloadPointer;
            this->headerPointer = other.headerPointer;
            this->setPointerData(other.headerPointer, other.payloadPointer);
            this->initialized = other.initialized;
            this->bskLogger = other.bskLogger;
            this->zeroMsgPayload = other.zeroMsgPayload;
            return *this;
        }

        messageType* incomingPayload = other.payloadPointer;
        MsgHeader* incomingHeader = other.headerPointer;
        bool incomingInitialized = other.initialized;
        BSKLogger incomingLogger = other.bskLogger;
        messageType incomingZeroPayload = other.zeroMsgPayload;
        void* incomingHandle = other.sourceHandle;
        void (*incomingAcquire)(void*) = other.acquireSource;
        void (*incomingRelease)(void*) = other.releaseSource;

        SourceReplacementGuard replacementGuard(this->replacingSource);
        SourceHandleGuard incomingOwner(incomingHandle,
                                        incomingAcquire,
                                        incomingRelease,
                                        true);
        this->payloadPointer = nullptr;
        this->headerPointer = nullptr;
        this->clearPointerData();
        this->initialized = false;
        this->releaseHandle_();

        this->bskLogger = std::move(incomingLogger);
        this->zeroMsgPayload = std::move(incomingZeroPayload);
        this->payloadPointer = incomingPayload;
        this->headerPointer = incomingHeader;
        this->setPointerData(incomingHeader, incomingPayload);
        this->initialized = incomingInitialized;
        this->sourceHandle = incomingHandle;
        this->acquireSource = incomingAcquire;
        this->releaseSource = incomingRelease;
        incomingOwner.relinquish();
        return *this;
    }

    //! move constructor -- steal the source token and null out the moved-from reader
    ReadFunctor(ReadFunctor&& other)
        : payloadPointer(other.payloadPointer),
          headerPointer(other.headerPointer),
          initialized(other.initialized),
          sourceHandle(other.sourceHandle),
          acquireSource(other.acquireSource),
          releaseSource(other.releaseSource),
          bskLogger(std::move(other.bskLogger)),
          zeroMsgPayload(std::move(other.zeroMsgPayload)) {
        this->setPointerData(this->headerPointer, this->payloadPointer);
        other.payloadPointer = nullptr;
        other.headerPointer = nullptr;
        other.clearPointerData();
        other.initialized = false;
        other.sourceHandle = nullptr;   // moved-from reader no longer owns the reference
        other.acquireSource = nullptr;
        other.releaseSource = nullptr;
    }

    //! move assignment -- release ours, steal theirs, null out the moved-from reader
    ReadFunctor& operator=(ReadFunctor&& other) {
        if (this == &other) { return *this; }
        if (this->replacingSource) { return *this; }

        messageType* incomingPayload = other.payloadPointer;
        MsgHeader* incomingHeader = other.headerPointer;
        bool incomingInitialized = other.initialized;
        BSKLogger incomingLogger = std::move(other.bskLogger);
        messageType incomingZeroPayload = std::move(other.zeroMsgPayload);
        void* incomingHandle = other.sourceHandle;
        void (*incomingAcquire)(void*) = other.acquireSource;
        void (*incomingRelease)(void*) = other.releaseSource;
        other.payloadPointer = nullptr;
        other.headerPointer = nullptr;
        other.clearPointerData();
        other.initialized = false;
        other.sourceHandle = nullptr;
        other.acquireSource = nullptr;
        other.releaseSource = nullptr;

        SourceReplacementGuard replacementGuard(this->replacingSource);
        SourceHandleGuard incomingOwner(incomingHandle,
                                        incomingAcquire,
                                        incomingRelease,
                                        false);
        this->payloadPointer = nullptr;
        this->headerPointer = nullptr;
        this->clearPointerData();
        this->initialized = false;
        this->releaseHandle_();

        this->bskLogger = std::move(incomingLogger);
        this->zeroMsgPayload = std::move(incomingZeroPayload);
        this->payloadPointer = incomingPayload;
        this->headerPointer = incomingHeader;
        this->setPointerData(incomingHeader, incomingPayload);
        this->initialized = incomingInitialized;
        this->sourceHandle = incomingHandle;
        this->acquireSource = incomingAcquire;
        this->releaseSource = incomingRelease;
        incomingOwner.relinquish();
        return *this;
    }

    //! Install (or replace) the opaque keep-alive source. Called only from the SWIG layer
    //! after a subscribe completes. The caller has already taken the one reference we adopt
    //! here (so we do NOT call acquire); subsequent C++ copies acquire, destructors release.
    void setSource(void* handle, void(*acquire)(void*), void(*release)(void*)) {
        if (this->replacingSource) {
            if (handle && release) { release(handle); }
            return;
        }
        if (handle == this->sourceHandle) {
            // Already holding this exact source (e.g. a re-subscribe to the same message).
            // The caller still took one reference we are contractually obliged to consume, so
            // drop it here rather than adopt a redundant one -- otherwise each re-subscribe
            // would leak a Py_INCREF that unsubscribe/destruction never balances (issue #676).
            if (handle && release) { release(handle); }
            return;
        }
        this->releaseHandle_();   // drop any prior owner
        this->sourceHandle = handle;
        this->acquireSource = acquire;
        this->releaseSource = release;
    }

    //! constructor
    const messageType& operator()(){
        if (!this->initialized) {
            messageType var;
            bskLogger.bskError("In C++ read functor, you are trying to read an un-connected message of type %s\nThis program is about to self destruct.",  typeid(var).name());
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
            bskLogger.bskError("In C++ read functor, you are checking if an unconnected msg of type %s is written.", typeid(var).name());
        }
    };

    //! return the time at which the message was written
    uint64_t timeWritten(){
        if (!this->initialized) {
            messageType var;
            bskLogger.bskError("In C++ read functor, you are requesting the write time of an unconnected msg of type %s.", typeid(var).name());
        }
        return this->headerPointer->timeWritten;
    };

    //! return the moduleID of who wrote wrote the message
    int64_t moduleID(){
        if (!this->initialized) {
            messageType var;
            bskLogger.bskError("In C++ read functor, you are requesting moduleID of an unconnected msg of type %s.", typeid(var).name());
        }
        if (!this->headerPointer->isWritten) {
            messageType var;
            bskLogger.bskError("In C++ read functor, you are requesting moduleID of an unwritten msg of type %s.", typeid(var).name());
        }
        return this->headerPointer->moduleID;
    };

    //! subscribe to a C message
    void subscribeToC(void* source){
        // this method works by knowing that the first member of a C message is the header.
        this->headerPointer = (MsgHeader*) source;

        // advance the address to connect to C-wrapped message payload
        // this assumes the header memory is aligned with 0 additional padding
        MsgHeader* pt = this->headerPointer;
        this->payloadPointer = (messageType *) (++pt);

        // set flag that this input message is connected to another message
        this->initialized = true;           // set input message as linked
        this->headerPointer->isLinked = 1;  // set source output message as linked

        this->setPointerData(this->headerPointer, this->payloadPointer);
    };
    //! Subscribe to the message located at the sourceAddr in memory
    void subscribeToAddr(uint64_t sourceAddr)
    {
        //!Cast a pointer at the sourceAddr and call the regular subscribe
        Message<messageType> *source =
            reinterpret_cast<Message<messageType> *> (sourceAddr);
	    subscribeTo(source);
    }
    //! Subscribe to the C message located at the sourceAddr in memory
    void subscribeToCAddr(uint64_t sourceAddr)
    {
        //! Cast a void* pointer and call the regular C-subscribe method
        void *source = reinterpret_cast<void *> (sourceAddr);
        subscribeToC(source);
    }
    //! Subscribe to a C++ message
    void subscribeTo(Message<messageType> *source){
        *this = source->addSubscriber();
    };

    //! Unsubscribe to the connected message, noop if no message was connected
    void unsubscribe(){
        if (this->replacingSource) { return; }

        SourceReplacementGuard replacementGuard(this->replacingSource);
        this->payloadPointer = nullptr;
        this->headerPointer = nullptr;
        this->clearPointerData();
        this->initialized = false;
        this->releaseHandle_();   // #676: drop the Python keep-alive reference, if any
    }

    //! Check if self has been subscribed to a C message
    uint8_t isSubscribedToC(void *source){

        int8_t firstCheck = (this->headerPointer == (MsgHeader*) source);
        MsgHeader* pt = this->headerPointer;
        int8_t secondCheck = (this->payloadPointer == (messageType *) (++pt));

        return (this->initialized && firstCheck && secondCheck);

    };
    //! Check if self has been subscribed to a Cpp message
    uint8_t isSubscribedTo(Message<messageType> *source){

        MsgHeader *dummyMsgPtr;
        int8_t firstCheck = (this->payloadPointer == source->getMsgPointers(&(dummyMsgPtr)));
        int8_t secondCheck = (this->headerPointer == dummyMsgPtr);

        return (this->initialized && firstCheck && secondCheck );

    };
    //! Check if self has been subscribed to the message at sourceAddr
    uint8_t isSubscribedToAddr(uint64_t sourceAddr)
    {
        //!Cast a pointer at the sourceAddr and call the regular is-subscribe
        Message<messageType> *source = reinterpret_cast<Message<messageType> *> (sourceAddr);
        return(isSubscribedTo(source));
    }
    //! Check if self has been subscribed to the message at sourceAddr
    uint8_t isSubscribedToCAddr(uint64_t sourceAddr)
    {
        //!Cast a void* pointer at the sourceAddr and call the regular is-subscribe
        void *source = reinterpret_cast<void *> (sourceAddr);
        return(isSubscribedToC(source));
    }

    //! Recorder method description
    Recorder<messageType> recorder(uint64_t timeDiff = 0){return Recorder<messageType>(this, timeDiff);}

    //! Return the address of the internal payloadPointer member
    uintptr_t getPayloadPtrAddress() const {
        return reinterpret_cast<uintptr_t>(&payloadPointer);
    }

    //! Return the address of the internal initialized flag
    uintptr_t getLinkedAddress() const {
        return reinterpret_cast<uintptr_t>(&initialized);
    }
};

/*! Write Functor */
template<typename messageType>
class WriteFunctor{
private:
    messageType* payloadPointer;    //!< pointer to the message payload
    MsgHeader* headerPointer;      //!< pointer to the message header
public:
    //! write functor constructor
    WriteFunctor(){};
    //! write functor constructor
    WriteFunctor(messageType* payloadPointer, MsgHeader *headerPointer) : payloadPointer(payloadPointer), headerPointer(headerPointer){};
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
class Message : public MessageBase{
private:
    messageType payload = {};   //!< struct defining message payload, zero'd on creation
    MsgHeader header = {};      //!< struct defining the message header, zero'd on creation
    ReadFunctor<messageType> read = ReadFunctor<messageType>(&payload, &header);  //!< read functor instance
public:
    Message();
    //! copy constructor
    Message(const Message& other);
    //! copy assignment operator
    Message& operator=(const Message& other);
    //! move constructor
    Message(Message&& other);
    //! move assignment operator
    Message& operator=(Message&& other);
    //! write functor to this message
    WriteFunctor<messageType> write = WriteFunctor<messageType>(&payload, &header);
    //! -- request read rights. returns reference to class ``read`` variable
    ReadFunctor<messageType> addSubscriber();
    //! -- request write rights.
    WriteFunctor<messageType> addAuthor();
    //! for plain ole c modules
    messageType* subscribeRaw(MsgHeader **msgPtr);

    //! for plain ole c modules
    messageType* getMsgPointers(MsgHeader **msgPtr);

    //! Recorder object
    Recorder<messageType> recorder(uint64_t timeDiff = 0){return Recorder<messageType>(this, timeDiff);}

    messageType zeroMsgPayload = {};    //!< zero'd copy of the message payload structure

    //! check if this msg has been connected to
    bool isLinked(){return this->header.isLinked;};

    //! Return the memory size of the payload, be careful about dynamically sized things
    uint64_t getPayloadSize() {return sizeof(messageType);};

    //! Return the raw address of the message payload struct
    uintptr_t getPayloadAddress() { return reinterpret_cast<uintptr_t>(&payload); }

    //! Return the raw address of the message header struct
    uintptr_t getHeaderAddress()  { return reinterpret_cast<uintptr_t>(&header);  }
};

template<typename messageType>
Message<messageType>::Message()
{
    this->setPointerData(&header, &payload);
}

template<typename messageType>
Message<messageType>::Message(const Message& other) : Message()
{
    *this = other;
}

template<typename messageType>
Message<messageType>& Message<messageType>::operator=(const Message& other)
{
    if (this != &other) {
        this->payload = other.payload;
        this->header = other.header;
        this->zeroMsgPayload = other.zeroMsgPayload;
        this->setPointerData(&this->header, &this->payload);
    }
    return *this;
}

template<typename messageType>
Message<messageType>::Message(Message&& other) : Message()
{
    *this = std::move(other);
}

template<typename messageType>
Message<messageType>& Message<messageType>::operator=(Message&& other)
{
    if (this != &other) {
        this->payload = std::move(other.payload);
        this->header = std::move(other.header);
        this->zeroMsgPayload = std::move(other.zeroMsgPayload);
        this->setPointerData(&this->header, &this->payload);
    }
    return *this;
}

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
messageType* Message<messageType>::subscribeRaw(MsgHeader **msgPtr){
    *msgPtr = &this->header;
    this->header.isLinked = 1;
    return &this->payload;
}

template<typename messageType>
messageType* Message<messageType>::getMsgPointers(MsgHeader **msgPtr){
    *msgPtr = &this->header;
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
        this->ModelTag = "Rec:" + findMsgName(std::string(typeid(*message).name()));
    }
    //! -- Use this to record C messages
    Recorder(void* message, uint64_t timeDiff = 0){
        this->timeInterval = timeDiff;

        // C messages store header followed immediately by payload in memory.
        // Advance past header to get payload pointer.
        auto* headerPtr  = static_cast<MsgHeader*>(message);
        auto* payloadPtr = reinterpret_cast<messageType*>(headerPtr + 1);

        this->readMessage = ReadFunctor<messageType>(payloadPtr, headerPtr);

        this->ModelTag = "Rec:";
        Message<messageType> tempMsg;
        std::string msgName = typeid(tempMsg).name();
        this->ModelTag += findMsgName(msgName);
    }
    //! -- Use this to keep track of what someone is reading
    Recorder(ReadFunctor<messageType>* messageReader, uint64_t timeDiff = 0){
        this->timeInterval = timeDiff;
        this->readMessage = *messageReader;
        if (!messageReader->isLinked()) {
            messageType var;
            bskLogger.bskError("In C++ read functor, you are requesting to record an un-connected input message of type %s.", typeid(var).name());
        }
        this->ModelTag = "Rec:" + findMsgName(std::string(typeid(*messageReader).name()));
    }
    //! -- Copy recorder state while keeping a unique SysModel module ID
    Recorder(const Recorder& obj) : SysModel() {
        this->copyRecorderState(obj);
    }
#ifndef SWIG
    //! -- Copy recorder state while preserving this recorder's module ID
    Recorder& operator=(const Recorder& obj) {
        if (this != &obj) {
            this->copyRecorderState(obj);
        }
        return *this;
    }
#endif
    ~Recorder(){};

    //! Install an opaque owner on the recorder's reader for the SWIG keep-alive bridge.
    void setSource(void* handle, void(*acquire)(void*), void(*release)(void*)) {
        this->readMessage.setSource(handle, acquire, release);
    }

    //! -- self initialization
    void SelfInit(){};
    //! -- cross initialization
    void IntegratedInit(){};
    //! -- Read and record the message
    void UpdateState(uint64_t CurrentSimNanos){
        if (CurrentSimNanos >= this->nextUpdateTime) {
            // Log warning if message is invalid but don't change behavior
            if (!this->readMessage.isLinked() || !this->readMessage.isWritten()) {
                this->logInvalidMessageWarning();
            }

            const messageType& messageData = this->readMessage();
            if (!this->recordOnlyOnChange || this->messagePayloadChanged(messageData)) {
                this->recordMessage(CurrentSimNanos, messageData);
            } else {
                this->scheduleNextUpdate(CurrentSimNanos);
            }
        }
    };
    //! Reset method
    void Reset(uint64_t CurrentSimNanos){
        this->msgRecord.clear();    //!< -- Can only reset to 0 for now
        this->msgRecordTimes.clear();
        this->msgWrittenTimes.clear();
        this->nextUpdateTime = CurrentSimNanos;
        this->hasLastUpdateTime = false;
    };
    //! recorded times, copied out as a std::vector so the Python interface is unchanged. The element
    //! type is "unsigned long long" (not uint64_t) to match the SWIG TimeVector template exactly; on
    //! LP64 platforms uint64_t is "unsigned long", a distinct type that would fail to copy-construct.
    std::vector<unsigned long long> times(){return std::vector<unsigned long long>(this->msgRecordTimes.begin(), this->msgRecordTimes.end());}
    //! message-written times, copied out as a std::vector (see times() for the element-type rationale)
    std::vector<unsigned long long> timesWritten(){return std::vector<unsigned long long>(this->msgWrittenTimes.begin(), this->msgWrittenTimes.end());}
    //! internal accessor: the recorded-payload deque by reference, used by the SWIG recorder typemaps
    std::deque<messageType>& record(){return this->msgRecord;};
    //! recorded payloads copied out as a std::vector, exposed to Python so the result is randomly indexable
    std::vector<messageType> recordList(){return std::vector<messageType>(this->msgRecord.begin(), this->msgRecord.end());}
    //! size of the record so far
    size_t size(){return this->msgRecord.size();}

    //! determine message name
    std::string findMsgName(std::string msgName) {
        size_t locMsg = msgName.find("Payload");
        if (locMsg != std::string::npos) {
            msgName.erase(locMsg, std::string::npos);
        }
        locMsg = msgName.find("Message");
        if (locMsg != std::string::npos) {
           msgName.replace(locMsg, 7, "");
        }
        for (int c = 0; c<10; c++) {
            locMsg = msgName.rfind(std::to_string(c));
            if (locMsg != std::string::npos) {
                msgName.erase(0, locMsg+1);
            }
        }
        return msgName;
    };

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
        if (this->hasLastUpdateTime) {
            this->nextUpdateTime = this->lastUpdateTime + this->timeInterval;
        }
    };

    //! method to record messages only when the payload content changes
    void recordOnChange(bool enabled = true) {
        if constexpr (!supportsPayloadEquality<messageType>()) {
            if (enabled) {
                bskLogger.bskLog(BSK_ERROR,
                    "Recorder::recordOnChange() called for a payload type that does not "
                    "support equality comparison. Add a PayloadEqualityTraits<messageType> "
                    "specialization in the payload header file (guarded by #ifdef __cplusplus), "
                    "or check whether generatePayloadEqualityHeader.py can cover all fields.");
                return;
            }
        }
        this->recordOnlyOnChange = enabled;
    };

private:
    std::deque<messageType> msgRecord;            //!< deque of recorded messages (deque avoids the geometric
                                                  //!< reallocation spikes a vector incurs as the history grows; see issue #788)
    std::deque<uint64_t> msgRecordTimes;          //!< deque of times at which messages are recorded
    std::deque<uint64_t> msgWrittenTimes;         //!< deque of times at which messages are written
    uint64_t nextUpdateTime = 0;                  //!< [ns] earliest time at which the msg is recorded again
    uint64_t timeInterval = 0;                    //!< [ns] recording time interval
    uint64_t lastUpdateTime = 0;                  //!< [ns] last time the msg was checked for recording
    bool hasLastUpdateTime = false;               //!< flag indicating whether the msg was checked
    bool recordOnlyOnChange = false;              //!< flag to record only changed message payloads

private:
    //! log warning if message is invalid but don't change behavior
    void logInvalidMessageWarning(){
        messageType var;
        bskLogger.bskLog(BSK_WARNING, "Recording message of type %s that is not properly initialized or written", typeid(var).name());
    };

    //! record the supplied message payload
    void recordMessage(uint64_t CurrentSimNanos, const messageType& messageData){
        this->msgRecordTimes.push_back(CurrentSimNanos);
        this->msgWrittenTimes.push_back(this->readMessage.timeWritten());
        this->msgRecord.push_back(messageData);
        this->scheduleNextUpdate(CurrentSimNanos);
    };

    //! schedule the next eligible recorder update
    void scheduleNextUpdate(uint64_t CurrentSimNanos){
        this->lastUpdateTime = CurrentSimNanos;
        this->hasLastUpdateTime = true;
        this->nextUpdateTime = CurrentSimNanos + this->timeInterval;
    };

    //! check if the payload differs from the last recorded payload
    bool messagePayloadChanged(const messageType& messageData){
        if constexpr (supportsPayloadEquality<messageType>()) {
            return this->msgRecord.empty() || !payloadsAreEqual(messageData, this->msgRecord.back());
        }
        return true;
    };

    ReadFunctor<messageType> readMessage;   //!< method description

    //! -- Copy recorder-owned state without copying the SysModel identity
    void copyRecorderState(const Recorder& obj){
        this->ModelTag = obj.ModelTag;
        this->RNGSeed = obj.RNGSeed;
        this->bskLogger = obj.bskLogger;
        this->msgRecord = obj.msgRecord;
        this->msgRecordTimes = obj.msgRecordTimes;
        this->msgWrittenTimes = obj.msgWrittenTimes;
        this->nextUpdateTime = obj.nextUpdateTime;
        this->timeInterval = obj.timeInterval;
        this->lastUpdateTime = obj.lastUpdateTime;
        this->hasLastUpdateTime = obj.hasLastUpdateTime;
        this->recordOnlyOnChange = obj.recordOnlyOnChange;
        this->readMessage = obj.readMessage;
    };
};

#endif
