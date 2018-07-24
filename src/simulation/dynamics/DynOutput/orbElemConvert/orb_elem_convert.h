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

#ifndef ORB_ELEM_CONVERT_H
#define ORB_ELEM_CONVERT_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "utilities/orbitalMotion.h"
#include "simMessages/spicePlanetStateSimMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief An orbital element/cartesian position and velocity converter
 The module
 [PDF Description](Basilisk-ORBELEMCONVERT-20170703.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */


class OrbElemConvert: public SysModel {
public:
    OrbElemConvert();
    ~OrbElemConvert();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    void Elements2Cartesian();
    void Cartesian2Elements();
    void ReadInputs();
    
public:
    double r_N[3];                    //!< m  Current position vector (inertial)
    double v_N[3];                    //!< m/s Current velocity vector (inertial)
    double mu;                        //!< -- Current grav param (inertial)
    classicElements CurrentElem;      //!< -- Current orbital elements
    SCPlusStatesSimMsg statesIn;
    SpicePlanetStateSimMsg planetIn;
    std::string StateString;          //!< -- port to use for conversion
    std::string OutputDataString;     //!< -- port to use for output data
    uint64_t OutputBufferCount;       //!< -- Count on number of buffers to output
	uint64_t stateMsgSize;            //!< -- Size of the state message to use
    bool ReinitSelf;                  //!< -- Indicator to reset conversion type
    bool Elements2Cart;               //!< -- Flag saying which direction to go
	bool useEphemFormat;              //!< -- Flag indicating whether to use state or ephem
    bool inputsGood;                  //!< -- flag indicating that inputs are good
    
private:
    int64_t StateInMsgID;              // -- MEssage ID for incoming data
    int64_t StateOutMsgID;             // -- Message ID for outgoing data
};

/*! @} */

#endif
