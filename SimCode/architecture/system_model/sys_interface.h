
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

class  InterfaceDataExchange{
public:
    InterfaceDataExchange();
    virtual ~InterfaceDataExchange();
    bool linkProcesses();
    bool linkMessages();
    void discoverMessages();
    void routeMessages();
public:
    bool exchangeActive;                //!< (-) Flag indicating that exchange is active
    MessageInterfaceMatch processData;  //!< Definition of process movement
    std::vector<MessageInterfaceMatch> messageTraffic; //!< Message movement
private:
    uint64_t msgBufferSize;
    uint8_t *msgBuffer;
};

//! Class used to pass messages between one or more processes
class SysInterface
{
    
public:
    SysInterface();
    ~SysInterface();
    void addNewInterface(std::string from, std::string to);
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
