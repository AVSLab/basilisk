/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/MotorStepCountMsg_C.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* Private variables */
    double currentMotorAngle;                                   //!< [rad] Curretn motor angle
    double desiredAngle;                                        //!< [rad] Desired motor angle 
    double deltaAngle;                                          //!< [rad] Difference between desired and current angle
    double stepAngle;                                           //!< [rad] Angle it takes for a single step to move
    double stepTime;                                            //!< [s] Time it takes for a motor to achieve 1 Step

    /* Steps variables */
    int stepCount;                                              //!< [steps] Total number of steps taken
    int stepsCommanded;                                         //!< [steps] Number of steps needed to reach the desired angle (output)
    int stepsTaken;                                             //!< [steps] The number of steps already achieved from the steps commanded


    // double Time;                                           
    // double callTime ;                                        //!< [s] It's from basilisk itself and it's the current time of simulation
    double previousWrittenTime ;                                //!< [s] Time the desired theta was given
    double deltaSimTime ;                                       //!< [s] The time we took to get a new message 

   
    BSKLogger* bskLogger;                                        //!< BSK Logging


    /* Messages */
    HingedRigidBodyMsg_C    spinningBodyInMsg;                   //!< Intput msg for the spinning body angle and angle rate
    MotorStepCountMsg_C    motorStepCountOutMsg;                 //!< Output msg for the number of commanded motor step counts

}StepperMotorConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_stepperMotor(StepperMotorConfig *configData, int64_t moduleID);                     //!< Method for module initialization
    void Reset_stepperMotor(StepperMotorConfig *configData, uint64_t callTime, int64_t moduleID);     //!< Method for module reset
    void Update_stepperMotor(StepperMotorConfig *configData, uint64_t callTime, int64_t moduleID);    //!< Method for module time update
#ifdef __cplusplus
}
#endif

#endif