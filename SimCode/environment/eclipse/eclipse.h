/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef Eclipse_H
#define Eclipse_H

#include <vector>
#include <map>
#include "_GeneralModuleFiles/sys_model.h"
#include "environment/spice/spice_planet_state.h"
#include "dynamics/spacecraftPlus/spacecraftPlusMsg.h"
#include "eclipse_data.h"
#include "utilities/linearAlgebra.h"

/*! \addtogroup SimModelGroup
 *  This group is used to model parts of the vehicle and the surrounding environment
 *  in the simulation system.  All components/dynamics/environment models are a
 *  part of this group.
 * @{
 */

//! The eclipse class gets generates sun illumination state at a
// particular inertial position
class Eclipse: public SysModel {
public:
    Eclipse();
    ~Eclipse();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void writeOutputMessages(uint64_t CurrentClock);
    std::string addPositionMsgName(std::string msgName);
    
public:
    uint64_t outputBufferCount; //!< -- Number of output buffers to use
    std::vector<std::string> planetNames;
    
private:
    std::vector<SpicePlanetState> planets;  //!< -- Names of planets we want to track
    std::map<uint64_t, std::string> planetInMsgIdAndName; //!< -- Internal vector of planets
    std::vector<std::string> positionMsgNames;  //!< -- vector of msg names for each position state for which to evaluate eclipse conditions.
    std::map<uint64_t, SCPlusOutputStateData> positionInMsgIdAndState;
    std::vector<uint64_t> eclipseOutMsgId;
    std::vector<std::string> eclipseOutMsgName;

private:
    void readInputMessages();
};

/*! @} */

#endif
