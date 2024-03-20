/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _SINGLEAXISPROFILER_
#define _SINGLEAXISPROFILER_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/StepperMotorMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Single Axis Profiler Class */
class SingleAxisProfiler: public SysModel {
public:
    SingleAxisProfiler() = default;                                    //!< Constructor
    ~SingleAxisProfiler() = default;                                   //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                     //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;               //!< Update member function
    void setRotHat_M(const Eigen::Vector3d &rotHat_M);                 //!< Setter for the spinning body rotation axis
    const Eigen::Vector3d &getRotHat_M() const;                        //!< Getter for the spinning body rotation axis

    ReadFunctor<StepperMotorMsgPayload> stepperMotorInMsg;             //!< Input msg for the stepper motor state information
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;    //!< Output msg for the hub-relative prescribed rotational states

    BSKLogger *bskLogger;                                              //!< BSK Logging

private:
    Eigen::Vector3d computeSigma_FM(double theta);                     //!< Method for computing the current spinning body MRP attitude relative to the mount frame: sigma_FM
    Eigen::Vector3d rotHat_M;                                          //!< Spinning body rotation axis expressed in M frame components

};

#endif
