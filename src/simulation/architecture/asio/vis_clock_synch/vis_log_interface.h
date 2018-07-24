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

#ifndef VIS_LOG_INTERFACE_H
#define VIS_LOG_INTERFACE_H

#include <string>
#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/message_logger.h"

/*! \addtogroup SimArchGroup
 * @{
 */

typedef struct {
    uint64_t simTimePrimary;
    std::vector<uint64_t> messagePoints;
}MessageTimeElement;

//!@brief The clock synchronization module is used to slave the simulation to realtime.
/*!  The module is controlled by specifying an acceleration factor which can be adjusted 
     dynamically if the timeInitialized factor is also reset dynamically.*/
class VisLogInterface: public SysModel {
public:
    VisLogInterface();
    ~VisLogInterface();
   
    void SelfInit();
    void CrossInit(); 
    void UpdateState(uint64_t CurrentSimNanos);
    bool loadDataFromFile(std::string dataFile);
    bool resolveDataMap();
    
public:
    messageLogger logData;
    std::vector<MessageTimeElement> messageDataMap;
private:
};

/*! @} */

#endif
