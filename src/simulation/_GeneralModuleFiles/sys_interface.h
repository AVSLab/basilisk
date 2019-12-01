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

#ifndef _SysInterface_HH_
#define _SysInterface_HH_

#include <vector>
#include <stdint.h>
#include "architecture/system_model/sys_model_task.h"
#include "utilities/bskLogging.h"
/*! \addtogroup SimArchGroup Simulation Architecture Classes
 * @{
 */

/*!
 * This struct is dual purpose. The comments in-line describe the use as it is in
 * the processData variable. Here, the block comment describes its used as it is in
 * messageTraffic:
 * std::string messageSource Name of message in source buffer
 * std::string messageDest Name of message in dest buffer (left blank i.e. same name)
 * int64_t source the ID for the message in its source buffer
 * int64_t destination The ID for the message in the destination buffer
 * uint64_t updateCounter the number of times a message had been written last time it was routed
 */
typedef struct {
    std::string messageSource;  //!< (-) name of process to get message from
    std::string messageDest;  //!< (-) name of process to put message in
    int64_t source;  //!< The ID for the buffer to copy messages from (negative indicates not ready)
    int64_t destination;  //!< The ID for the buffer to copy messages to (negative indicates not ready)
    uint64_t updateCounter;  //!< (-) The number of times a message had been written last time it was routed.
}MessageInterfaceMatch;  //!< -- Routing information for a specific message

/*!
 * This class encapsulates the interface from a single source to a single destination.
 * A SysInterface has a list of these.
 */
class  InterfaceDataExchange : public SysModel{
public:
    InterfaceDataExchange();
    virtual ~InterfaceDataExchange();
    virtual bool linkProcesses();  //!<  uses the input process names to
    virtual bool linkMessages();  //!< get read/write permission in source/destination buffers
    virtual void discoverMessages();  //!< looks for unpublished messages in the buffer and adds them to the list
    virtual void routeMessages();  //!< copy message data from source to destination
public:
    bool exchangeActive;  //!< (-) Flag indicating that exchange is active
    MessageInterfaceMatch processData;  //!< Definition of process movement
    std::vector<MessageInterfaceMatch> messageTraffic;  //!< Message movement
    bool needDelete;  //!< Used by destructor to clear memory for this exchange
    BSKLogger bskLogger;                      //!< -- BSK Logging
private:
    uint64_t msgBufferSize;  //!< size of message in bytes currently being routed
    uint8_t *msgBuffer;
};

/*!
 * This class is used to organize and manage all of the interfaces for a specific process. Like an interface manager.
 * It maintains a list of InterfaceDataExchange. For instance,
 */
class SysInterface
{
    
public:
    SysInterface();
    virtual ~SysInterface();
    void addNewInterface(std::string from, std::string to, std::string intName="");
    void addNewInterface(InterfaceDataExchange  *newInterface);
    //void addPassToCurrent(std::string messagePass);
    void enableInterface() { interfaceActive = true; }
    void disableInterface() { interfaceActive = false; }
    virtual void routeInputs(int64_t processBuffer);
    void discoverAllMessages();
    void connectInterfaces();

public:
    std::vector<InterfaceDataExchange *> interfaceDef; //!< List of interfaces
    bool interfaceActive;                           //!< -- Flag indicate whether interface has been disabled
    bool interfacesLinked;                          //!< (-) Flag indicating ints have all been linked
    BSKLogger bskLogger;                      //!< -- BSK Logging
private:
    InterfaceDataExchange *currentInterface;        //!< (-) allows user to get/set msgs for single int
};

/*! @} */
#endif /* _SysInterface_H_ */
