/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.
 @return void
 */
TabularAtmosphere::TabularAtmosphere()
{
    return;
}

/*! Empty destructor method.
 @return void
 */
TabularAtmosphere::~TabularAtmosphere()
{
    return;
}

void TabularAtmosphere::customReset(uint64_t CurrentClock)
{
    this->altList_length = this->altList.size();
    this->rhoList_length = this->rhoList.size();
    this->tempList_length = this->tempList.size();
    

    if((this->altList_length == this->rhoList_length) && (this->altList_length == this->tempList_length)){
        return;
    } else {
        std::cout << "Error: data not equal length." << std::endl;
    }
    
    if(this->altList_length == 0){
        std::cout << "Error: no data in altitude list." << std::endl;
    } else if(this->rhoList_length == 0){
        std::cout << "Error: no data in density list." << std::endl;
    } else if(this->tempList_length == 0){
        std::cout << "Error: no data in temperature list." << std::endl;
    }
    
    return;
}

void TabularAtmosphere::evaluateAtmosphereModel(AtmoPropsMsgPayload *msg, double currentTime)
{
    for(int i=0; i <= this->altList.size() - 1; i++){
		if(this->altList[i] > this->orbitAltitude){
			msg->neutralDensity = this->rhoList[i-1] + (this->orbitAltitude - this->altList[i-1]) * (this->rhoList[i] - this->rhoList[i-1]) / (this->altList[i] - this->altList[i-1]);
			msg->localTemp = this->tempList[i-1] + (this->orbitAltitude - this->altList[i-1]) * (this->tempList[i] - this->tempList[i-1]) / (this->altList[i] - this->altList[i-1]);
			break;
        }
    }
    return;
}
