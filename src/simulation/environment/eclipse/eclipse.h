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

#ifndef Eclipse_H
#define Eclipse_H

#include <vector>
#include <Eigen/Dense>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief eclipse model class */
class Eclipse: public SysModel {
public:
    Eclipse();
    ~Eclipse();
    
    void Reset(uint64_t CurrenSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void writeOutputMessages(uint64_t CurrentClock);
    void addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg);
    void addPlanetToModel(Message<SpicePlanetStateMsgPayload> *tmpSpMsg);
    
public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;   //!< sun ephemeris input message name
    std::vector<ReadFunctor<SpicePlanetStateMsgPayload>> planetInMsgs;  //!< A vector of planet incoming state message names ordered by the sequence in which planet are added to the module
    std::vector<ReadFunctor<SCStatesMsgPayload>> positionInMsgs;  //!< vector of msgs for each spacecraft position state for which to evaluate eclipse conditions.
    std::vector<Message<EclipseMsgPayload>*> eclipseOutMsgs;//!< vector of eclispe output msg names
    BSKLogger bskLogger;                        //!< BSK Logging
    double rEqCustom;  //!< [m] Custom radius

private:
    std::vector<float> planetRadii; //!< [m] A vector of planet radii ordered by the sequence in which planet names are added to the module
    std::vector<SCStatesMsgPayload> scStateBuffer;      //!< buffer of the spacecraft state input messages
    std::vector<SpicePlanetStateMsgPayload> planetBuffer;   //!< buffer of the spacecraft state input messages
    SpicePlanetStateMsgPayload sunInMsgState;               //!< copy of sun input msg
    std::vector<double> eclipseShadowFactors;               //!< vector of shadow factor output values

private:
    void readInputMessages();
    double computePercentShadow(double planetRadius, Eigen::Vector3d r_HB_N, Eigen::Vector3d s_BP_N);
    double getPlanetEquatorialRadius(std::string planetSpiceName);

};


#endif
