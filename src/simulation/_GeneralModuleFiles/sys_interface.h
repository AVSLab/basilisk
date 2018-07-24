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
/*! \addtogroup SimArchGroup
 * @{
 */

typedef struct {
    std::string messageSource;           //!< (-) source name used to find ID
    std::string messageDest;             //!< (-) dest name used to find ID
    int64_t source;       //!< The buffer to copy messages from (negative indicates not ready)
    int64_t destination;         //!< The buffer to copy messages to (negative indicates not ready)
    uint64_t updateCounter;  //!< (-) The previous update counter that was routed
}MessageInterfaceMatch;

class  InterfaceDataExchange : public SysModel{
public:
    InterfaceDataExchange();
    virtual ~InterfaceDataExchange();
    virtual bool linkProcesses();
    virtual bool linkMessages();
    virtual void discoverMessages();
    virtual void routeMessages();
public:
    bool exchangeActive;                //!< (-) Flag indicating that exchange is active
    MessageInterfaceMatch processData;  //!< Definition of process movement
    std::vector<MessageInterfaceMatch> messageTraffic; //!< Message movement
    bool needDelete;
private:
    uint64_t msgBufferSize;
    uint8_t *msgBuffer;
};

//! Class used to pass messages between one or more processes
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
    virtual void routeInputs(uint64_t processBuffer);
    void discoverAllMessages();
    void connectInterfaces();
    
public:
    std::vector<InterfaceDataExchange *> interfaceDef; //!< Definition of messages to move
    bool interfaceActive;                           //!< -- Flag indicate whether Task has been disabled
    bool interfacesLinked;                          //!< (-) Flag indicating ints have all been linked
private:
    InterfaceDataExchange *currentInterface;        //!< (-) allows user to get/set msgs for single int
};

/*! @} */
#endif /* _SysInterface_H_ */
