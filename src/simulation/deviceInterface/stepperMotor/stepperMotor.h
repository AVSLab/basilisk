/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
class StepperMotor : public SysModel {
   public:
    StepperMotor() = default;   //!< Constructor
    ~StepperMotor() = default;  //!< Destructor

    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;
    double getThetaInit() const;
    double getStepAngle() const;
    double getStepTime() const;
    double getThetaDDotMax() const;
    void setThetaInit(const double thetaInit);
    void setStepAngle(const double stepAngle);
    void setStepTime(const double stepTime);

    ReadFunctor<MotorStepCommandMsgPayload>
        motorStepCommandInMsg;                           //!< Input msg for the number of commanded motor step counts
    Message<StepperMotorMsgPayload> stepperMotorOutMsg;  //!< Output msg for the stepper motor state information

   private:
    void actuateMotor(double t);
    void resetMotor();
    void updateStepParameters();
    bool isInStepFirstHalf(double t);
    void computeStepFirstHalf(double t);
    bool isInStepSecondHalf(double t);
    void computeStepSecondHalf(double t);
    void computeStepComplete(double t);

    /* Step parameters */
    double stepAngle{};    //!< [rad] Angle the stepper motor moves through for a single step (constant)
    double stepTime{};     //!< [s] Time required for a single motor step (constant)
    int stepsCommanded{};  //!< [steps] Number of commanded steps
    int stepCount{};       //!< [steps] Current motor step count (number of steps taken)

    /* Motor angle parameters */
    double thetaInit{};              //!< [rad] Initial motor angle
    double intermediateThetaInit{};  //!< [rad] Initial motor angle at the start of each step
    double intermediateThetaRef{};   //!< [rad] Reference motor angle at the end of each step
    double theta{};                  //!< [rad] Current motor angle
    double thetaDot{};               //!< [rad/s] Current motor angle rate
    double thetaDDot{};              //!< [rad/s^2] Current motor angular acceleration
    double thetaDDotMax{};           //!< [rad/s^2] Maximum angular acceleration of the stepper motor

    /* Temporal parameters */
    double tInit{};                //!< [s] Simulation time at the beginning of each step
    double previousWrittenTime{-1};  //!< [ns] Time the last input message was written
    double ts{};                   //!< [s] The simulation time halfway through each step (switch time for ang accel)
    double tf{};                   //!< [s] Simulation time when the current step will be completed

    /* Boolean parameters */
    bool actuationComplete{true};  //!< Boolean designating the motor has fully completed the commanded actuation
    bool stepComplete{true};       //!< Boolean designating the completion of a step
    bool newMsg{};             //!< Boolean designating a new command message is written
    bool interruptMsg{};       //!< Boolean designating the new command message is interrupting motor actuation

    /* Constant parameters */
    double a{};  //!< Parabolic constant for the first half of a step
    double b{};  //!< Parabolic constant for the second half of a step
};

#endif
