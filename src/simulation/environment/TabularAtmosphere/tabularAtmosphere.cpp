/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "tabularAtmosphere.h"
#include "architecture/utilities/linearAlgebra.h"
#include <iostream>

/*! The constructor method initializes data list lengths to zero.

 */
TabularAtmosphere::TabularAtmosphere()
{
    // initialize to avoid compiler warnings
    this->altList_length = 0;
    this->rhoList_length = 0;
    this->tempList_length = 0;
    return;
}

/*! Empty destructor method.

 */
TabularAtmosphere::~TabularAtmosphere()
{
    return;
}

/*! Reset method checks that the data lists for altitude, density, and temperature have been defined with equal nonzero lengths.
*
*/
void TabularAtmosphere::customReset(uint64_t CurrentClock)
{
    this->altList_length = (int) this->altList.size();
    this->rhoList_length = (int) this->rhoList.size();
    this->tempList_length = (int) this->tempList.size();


    if((this->altList_length != this->rhoList_length) || (this->altList_length != this->tempList_length)){
        bskLogger.bskLog(BSK_ERROR, "Input arrays not of equal length.");
    }

    if(this->altList_length == 0){
        bskLogger.bskLog(BSK_ERROR, "No data in altitude list.");
    } else if(this->rhoList_length == 0){
        bskLogger.bskLog(BSK_ERROR, "No data in density list.");
    } else if(this->tempList_length == 0){
        bskLogger.bskLog(BSK_ERROR, "No data in temperature list.");
    }

    return;
}

/*! evaluate function interpolates from given data lists. Sets density and temp to 0 if altitude outside bounds of input lists OR if outside bounds of envMinReach and envMaxReach.
*
*/
void TabularAtmosphere::evaluateAtmosphereModel(AtmoPropsMsgPayload *msg, double currentTime)
{
    if ((this->orbitAltitude < this->altList[0]) || (this->orbitAltitude > this->altList.back())) {
        msg->neutralDensity = 0.0;
        msg->localTemp = 0.0;
    }
    else {
        for (uint32_t i = 0; i <= this->altList.size() - 1; i++) {
            if (this->altList[i] > this->orbitAltitude) {
                msg->neutralDensity = this->rhoList[i - 1] + (this->orbitAltitude - this->altList[i - 1]) * (this->rhoList[i] - this->rhoList[i - 1]) / (this->altList[i] - this->altList[i - 1]);
                msg->localTemp = this->tempList[i - 1] + (this->orbitAltitude - this->altList[i - 1]) * (this->tempList[i] - this->tempList[i - 1]) / (this->altList[i] - this->altList[i - 1]);
                break;
            }
        }
    }
    return;
}
