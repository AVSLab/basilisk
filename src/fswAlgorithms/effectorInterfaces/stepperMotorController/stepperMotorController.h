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

#ifndef _STEPPERMOTORCONTROLLER_
#define _STEPPERMOTORCONTROLLER_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/MotorStepCommandMsg_C.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include <cstdint>

/*! @brief Stepper Motor Controller Class */
class StepperMotorController: public SysModel{
public:

    StepperMotorController() = default;                                    //!< Constructor
    ~StepperMotorController() = default;                                   //!< Destructor

    void Reset(uint64_t currentSimNanos) override;                         //!< Reset member function
    void UpdateState(uint64_t currentSimNanos) override;                   //!< Update member function
    double getThetaInit() const;                                           //!< Getter method for the initial motor angle
    double getStepAngle() const;                                           //!< Getter method for the motor step angle
    double getStepTime() const;                                            //!< Getter method for the motor step time
    void setThetaInit(const double thetaInit);                             //!< Setter method for the initial motor angle
    void setStepAngle(const double stepAngle);                             //!< Setter method for the motor step angle
    void setStepTime(const double stepTime);                               //!< Setter method for the motor step time

    ReadFunctor<HingedRigidBodyMsgPayload> motorRefAngleInMsg;             //!< Intput msg for the stepper motor reference message
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;            //!< Output msg for the number of commanded motor step counts

    BSKLogger *bskLogger;                                                  //!< BSK Logging

private:

    /* Motor angle parameters */
    double thetaInit{};                                                    //!< [rad] Initial motor angle
    double theta{};                                                        //!< [rad] Current motor angle
    double thetaRef{};                                                     //!< [rad] Motor reference angle
    double deltaTheta{};                                                   //!< [rad] Difference between desired and current angle
    double stepAngle{};                                                    //!< [rad] Angle the stepper motor moves through for a single step (constant)

    /* Step parameters */
    int stepsCommanded{};                                                  //!< [steps] Number of steps needed to reach the desired angle (output)
    int stepCount{};                                                       //!< [steps] Current motor step count (number of steps taken)

    /* Temporal parameters */
    double stepTime{1.0};                                                  //!< [s] Time required for a single motor step (constant)
    double previousWrittenTime{-1.0};                                      //!< [ns] Time the last input message was written
    double deltaSimTime{};                                                 //!< [ns] The time elapsed since the last message was written
};

#endif
