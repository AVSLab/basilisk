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

#ifndef BORE_ANG_CALC_H
#define BORE_ANG_CALC_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/boreAngleSimMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief A class to perform a range of boresight related calculations.
 The module
 [PDF Description](Basilisk-BOREANGLECALC-20170722.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

class BoreAngCalc: public SysModel {
public:
    BoreAngCalc();
    ~BoreAngCalc();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void computeAxisPoint();
    void computeOutputData();
    void WriteOutputMessages(uint64_t CurrentClock);
    void ReadInputs();
    
public:
    std::string StateString;          //!< (-) port to use for conversion
    std::string celBodyString;        //!< (-) celestial body we are pointing at
    std::string OutputDataString;     //!< (-) port to use for output data
    uint64_t OutputBufferCount;       //!< (-) Count on number of buffers to output
    bool ReinitSelf;                  //!< (-) Indicator to reset conversion type
    double boreVec_B[3];              //!< (-) boresight vector in structure
    double boreVecPoint[3];           //!< (-) pointing vector in the target relative point frame
    AngOffValuesSimMsg boresightAng; //!< (-) Boresigt angles relative to target
    bool inputsGood;                  //!< (-) Flag indicating that inputs were read correctly
    
private:
    SpicePlanetStateSimMsg localPlanet;//!< (-) planet that we are pointing at
    SCPlusStatesSimMsg localState;   //!< (-) observed state of the spacecraft
    int64_t StateInMsgID;             // (-) MEssage ID for incoming data
    int64_t celInMsgID;               // (-) MEssage ID for incoming data
    int64_t AngOutMsgID;              // (-) Message ID for outgoing data
};

/*! @} */

#endif
