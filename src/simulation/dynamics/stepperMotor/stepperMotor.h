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

    ReadFunctor<MotorStepCommandMsgPayload> motorStepCommandInMsg;    //!< Input msg for the number of commanded motor step counts
    Message<StepperMotorMsgPayload> stepperMotorOutMsg;               //!< Output msg for the stepper motor state information

private:

    void actuateMotor(uint64_t callTime);                             //!< High-level method used to simulate the stepper motor states in time
    void resetMotor(double t);                                        //!< Method used to reset the motor states when the current request is complete and a new request is received
    void updateRotationParameters();                                  //!< Method used to update the rotation parameters after a step is completed
    bool isInStepFirstHalf(double t);                                 //!< Method used to determine if the motor is in the first half of a step
    void computeStepFirstHalf(double t);                              //!< Method used to compute the motor states during the first half of each step
    bool isInStepSecondHalf(double t);                                //!< Method used to determine if the motor is in the second half of a step
    void computeStepSecondHalf(double t);                             //!< Method used to compute the motor states during the second half of each step
    void computeStepComplete(double t);                               //!< Method used to compute the motor states when a step is complete

    /* Step parameters */
    double stepAngle;                                                 //!< [rad] Angle the stepper motor moves through for a single step
    double stepTime;                                                  //!< [s] Time required for a single motor step (constant)
    int stepsCommanded;                                               //!< [steps] Number of commanded steps
    int stepCount;                                                    //!< [steps] Current motor step count (number of steps taken)

    /* Motor angle parameters */
    double thetaInit;                                                 //!< [rad] Initial motor angle
    double maneuverThetaInit;                                         //!< [rad] Initial motor angle
    double intermediateThetaInit;                                     //!< [rad] Motor angle at the start of a new maneuver
    double intermediateThetaRef;                                      //!< [rad] Motor angle at the end of each step
    double theta;                                                     //!< [rad] Current motor angle
    double thetaDot;                                                  //!< [rad/s] Current motor angle rate
    double thetaDDot;                                                 //!< [rad/s^2] Current motor angular acceleration
    double thetaDDotMax;                                              //!< [rad/s^2] Maximum angular acceleration of the stepper motor

    /* Temporal parameters */
    double tInit;                                                     //!< [s] Simulation time at the beginning of the maneuver
    double previousWrittenTime;                                       //!< [ns] Time the last input message was written
    double ts;                                                        //!< [s] The simulation time halfway through the maneuver (switch time for ang accel)
    double tf;                                                        //!< [s] Simulation time when the maneuver is finished

    /* Boolean parameters */
    bool completion;                                                  //!< Boolean designating a fully completed maneuver
    bool stepComplete;                                                //!< Boolean designating a completed step
    bool newMsg;                                                      //!< Boolean designating a new message was written

    /* Constant parameters */
    double a;                                                         //!< Parabolic constant for the first half of a step
    double b;                                                         //!< Parabolic constant for the second half of a step
};

#endif
