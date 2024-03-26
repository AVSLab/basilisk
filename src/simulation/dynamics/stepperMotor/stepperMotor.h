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

#ifndef _STEPPERMOTOR_
#define _STEPPERMOTOR_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/MotorStepCommandMsgPayload.h"
#include "architecture/msgPayloadDefC/StepperMotorMsgPayload.h"
#include <stdint.h>

/*! @brief Stepper motor class. */
class StepperMotor: public SysModel {
public:

    StepperMotor() = default;                                         //!< Constructor
    ~StepperMotor() = default;                                        //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                    //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;              //!< Update member function
    double getThetaInit() const;                                      //!< Getter method for the initial motor angle
    double getStepAngle() const;                                      //!< Getter method for the motor step angle
    double getStepTime() const;                                       //!< Getter method for the motor step time
    double getThetaDDotMax() const;                                   //!< Getter method for the maximum motor angular acceleration
    void setThetaInit(const double thetaInit);                        //!< Setter method for the initial motor angle
    void setStepAngle(const double stepAngle);                        //!< Setter method for the motor step angle
    void setStepTime(const double stepTime);                          //!< Setter method for the motor step time
    void setThetaDDotMax(const double thetaDDotMax);                  //!< Setter method for the maximum motor angular acceleration

    BSKLogger *bskLogger;                                             //!< BSK Logging

    /* Messages */
    ReadFunctor<MotorStepCommandMsgPayload> motorStepCommandInMsg;    //!< Input msg for the number of commanded motor step counts
    Message<StepperMotorMsgPayload> stepperMotorOutMsg;               //!< Output msg for the stepper motor information

private:

    double thetaInit;                               //!< [rad] Initial motor angle
    double stepAngle;                               //!< [rad] Angle the stepper motor moves through for a single step
    double stepTime;                                //!< [s] Time required for a single motor step (constant)
    double thetaDDotMax;                            //!< [rad/s^2] Maximum angular acceleration of the stepper motor

    /* Step parameters */
    int stepsCommanded;                             //!< [steps] Number of commanded steps
    int stepCount;                                  //!< [steps] Current motor step count (number of steps taken)

    /* Motor angle parameters */
    double maneuverThetaInit;                       //!< [rad] Initial motor angle
    double intermediateThetaInit;                   //!< [rad] Motor angle at the start of a new maneuver
    double thetaDotInit;                            //!< [rad/s] Initial motor angle rate
    double intermediateThetaRef;                    //!< [rad] Motor angle at the end of each step
    double thetaDotRef;                             //!< [rad/s] Reference angle rate
    double theta;                                   //!< [rad] Current motor angle
    double thetaDot;                                //!< [rad/s] Current motor angle rate
    double thetaDDot;                               //!< [rad/s^2] Current motor angular acceleration

    /* Temporal parameters */
    double tInit;                                   //!< [s] Simulation time at the beginning of the maneuver
    double previousWrittenTime;                     //!< [ns] Time the last input message was written
    double ts;                                      //!< [s] The simulation time halfway through the maneuver (switch time for ang accel)
    double tf;                                      //!< [s] Simulation time when the maneuver is finished

    /* Boolean parameters */
    bool completion;                                //!< Boolean designating a fully completed maneuver
    bool stepComplete;                              //!< Boolean designating a completed step
    bool newMsg;                                    //!< Boolean designating a new message was written

    /* Constant parameters */
    double a;                                       //!< Parabolic constant for the first half of a step
    double b;                                       //!< Parabolic constant for the second half of a step
};

#endif
