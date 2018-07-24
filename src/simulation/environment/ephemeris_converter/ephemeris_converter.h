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

#ifndef EPHEMERIS_CONVERTER_H
#define EPHEMERIS_CONVERTER_H

#include <map>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/idEphemerisSimMsg.h"
#include "simFswInterfaceMessages/ephemerisIntMsg.h"


/*!@brief This class is used to take ephemeris data from the environmental models
        and convert it over to a FSW representation so that the ephemeris from 
        SPICE can be patched into the FSW directly instead of generating data 
        from an ephemeris model.

 The module
 [PDF Description](Basilisk-EPHEMERIS_CONVERTER20170712.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

*/
class EphemerisConverter: public SysModel {
public:
    EphemerisConverter();
    ~EphemerisConverter();
    
    bool LinkMessages();
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void readInputMessages();
    void convertEphemData(uint64_t clockNow);
    void writeOutputMessages(uint64_t Clock);
    
public:
    bool messagesLinked;        //!< [-] Flag used to determine if messages are cross-linked
    std::map<std::string, std::string> messageNameMap;   //!< [-] Map between input/output message names
    uint64_t numOutputBuffers;  //!< [-] Number of output buffers created for messages
private:
    std::map<int64_t, IDEphemerisSimMsg> messageIDMap; //!< [-] Map between input/output message IDs
};

#endif
