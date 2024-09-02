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

#ifndef planetEphemeris_H
#define planetEphemeris_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief planet ephemeris class */
class PlanetEphemeris: public SysModel {
public:
    PlanetEphemeris();
    ~PlanetEphemeris();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    void setPlanetNames(std::vector<std::string> planetNames);

public:
    std::vector<Message<SpicePlanetStateMsgPayload>*> planetOutMsgs; //!< -- vector of planet state output messages

    std::vector<ClassicElements>planetElements; //!< -- Vector of planet classical orbit elements

    std::vector<double> rightAscension;         //!< [r] right ascension of the north pole rotation axis (pos. 3-axis)
    std::vector<double> declination;            //!< [r] Declination of the north pole rotation axis (neg. 2-axis)
    std::vector<double> lst0;                   //!< [r] initial planet local sidereal time angle (pos. 3-axis)

    std::vector<double> rotRate;                //!< [r/s] planet rotation rate

    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    std::vector<std::string> planetNames;       //!< -- Vector of planet names
    double epochTime;                           //!< [s] time of provided planet ephemeris epoch
    int computeAttitudeFlag;                    //!< -- flag indicating if the planet orienation information is provided
};


#endif
