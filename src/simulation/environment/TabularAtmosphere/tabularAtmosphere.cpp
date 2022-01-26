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

    altList_length = altList.size();
    rhoList_length = rhoList.size();
    tempList_length = tempList.size();
    

    if((altList_length == rhoList_length) && (altList_length == tempList_length)){
        return;
    } else {
        printf("Error: data not equal length.");
    }

    return;
}

/*! Empty destructor method.
 @return void
 */
TabularAtmosphere::~TabularAtmosphere()
{
    return;
}

void TabularAtmosphere::evaluateAtmosphereModel(AtmoPropsMsgPayload *msg, double currentTime)
{
    // printf("Orbit Altitude = %.2f \n ",this->orbitAltitude);
    std::cout << "altList:" << altList.size() << std::endl;
    std::cout << currentTime << std::endl;
    for(int i=1; i <= this->altList.size(); i++){
        // printf("%.2f\n", altList[i]);
        // printf("%.2f\n", rhoList[i]);
		if(this->altList[i] > this->orbitAltitude){
			msg->neutralDensity = this->rhoList[i-1] + (this->orbitAltitude - this->altList[i-1]) * (this->rhoList[i] - this->rhoList[i-1]) / (this->altList[i] - this->altList[i-1]);
			msg->localTemp = this->tempList[i-1] + (this->orbitAltitude - this->altList[i-1]) * (this->tempList[i] - this->tempList[i-1]) / (this->altList[i] - this->altList[i-1]);
        }
    }
    
    // printf("returned value: %.2f\n\n", msg->neutralDensity);
    
    return;
}
