/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef HINGEDBODYLINEARPROFILER_H
#define HINGEDBODYLINEARPROFILER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief Linear deployment profiler for single hinged rigid body.
 */
class HingedBodyLinearProfiler: public SysModel {
public:
    HingedBodyLinearProfiler();
    ~HingedBodyLinearProfiler();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    uint64_t startTime; //!< [ns] time to begin deployment
    uint64_t endTime; //!< [ns] time to end deployment
    double startTheta; //!< [rad] starting hinged rigid body theta position
    double endTheta; //!<  [rad] ending hinged rigid body theta position

    Message<HingedRigidBodyMsgPayload> hingedRigidBodyReferenceOutMsg;  //!< -- output message for reference hinged rigid body state (theta, theta dot)

    BSKLogger bskLogger;              //!< -- BSK Logging

private:
    double deploymentSlope; //!<  [rad/s] slope of deployment
};


#endif
