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

/* Import the module header file */
#include "stepperMotor.h"

/* Other required files to import */
#include <stdbool.h>
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_stepperMotor(StepperMotorConfig* configData, int64_t moduleID)
{
    // Initialize the output messages
    MotorStepCountMsg_C_init(&configData->motorStepCountOutMsg);
}


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_stepperMotor(StepperMotorConfig* configData, uint64_t callTime, int64_t moduleID)
{
    // Check if the required input message is linked
    if (!HingedRigidBodyMsg_C_isLinked(&configData->spinningBodyInMsg))
    {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: stepperMotor.spinningBodyInMsg wasn't connected.");
    }

    // Calculate the angle 
    

    // Set the initial step count to zero
    configData->stepCount = 0;

    // Set the initial delta theta by getting the differenc beetween desired and current angle
    configData->deltaAngle = 0.0;

    // Set the initial number of stepps commanded
    configData->stepsCommanded = 0;

    // Set the initial time in second to know diference between the time we got a messsage to the current time we are in
    configData->deltaSimTime = 0.0;     

    // Set the initial number of steps already achieved from the steps commanded
    configData->stepsTaken = 0;          

    // Set the initial time were desired theta message was given. It is assigned to be -1 because we don't want its condition to overlap with the call time as we can recieve a message at call time 0.0, where call time is an integer that can't be negative
    configData->previousWrittenTime = -1.0;    
}


/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] The current time of simulation
 @param moduleID The module identifier
*/
void Update_stepperMotor(StepperMotorConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Create the buffer messages
    MotorStepCountMsgPayload motorStepCountOut;
    HingedRigidBodyMsgPayload spinningBodyIn;

    // Zero the output messages
    motorStepCountOut = MotorStepCountMsg_C_zeroMsgPayload();

    // Read the input message
    spinningBodyIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&configData->spinningBodyInMsg))
    {
        spinningBodyIn = HingedRigidBodyMsg_C_read(&configData->spinningBodyInMsg);
    }

    // Check if we have a new message of desired angle to excute the number of steps commanded (no interaption) 
    if ((configData->previousWrittenTime) <  HingedRigidBodyMsg_C_timeWritten(&configData->spinningBodyInMsg))
    {   
        // Assign the previous time to be the new written time
        configData->previousWrittenTime = HingedRigidBodyMsg_C_timeWritten(&configData->spinningBodyInMsg);

        // Assign the input theta as the desired theta
        configData->desiredAngle = spinningBodyIn.theta;

        // Calculate the delta angle
        configData->deltaAngle = (configData->desiredAngle) - (configData->currentMotorAngle);

        // Calculate the number of stepps commanded      
        configData->stepsCommanded = (configData->deltaAngle) / (configData->stepAngle);

        // Output the stepps commanded message to excute the manuevere
        motorStepCountOut.numSteps = configData->stepsCommanded; 

        // Zero steps taken in case of an interaption and start over 
        configData->stepsTaken = 0.0; 
    } 
        //while the stepps commanded are excuting we need to calculate the steps taken and the steps requested and the time it
        //took to reach steps taken in order to know how many steps are lift to complete the execution

    // Assure that the steps calculated are in the arthematic sequence of our step angle 
    if (configData->stepsCommanded - configData->stepsTaken != 0)
    {
        // Calculate the time taken from the start time of message given to the current time
        configData->deltaSimTime = (callTime) - (configData->previousWrittenTime);     
        
        // Calculate the number of steps already achieved from the steps commanded: 
        configData->stepsTaken = ceil(configData->deltaSimTime / configData->stepTime);                     

        //Once steps commanded are achieved, we will update the step count
        configData->stepCount = (configData->stepCount) + (configData->stepsTaken);

        // Update the current angle with each step 
        configData->currentMotorAngle = (configData->stepsTaken) * (configData->stepAngle);
    }
    MotorStepCountMsg_C_write(&motorStepCountOut, &configData->motorStepCountOutMsg, moduleID, callTime);
}    