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
#include <inttypes.h>

/* Other required files to import */
#include <stdbool.h>
#include  <stdlib.h>
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
    configData->deltaSimTime = 0;     

    // Set the initial number of steps already achieved from the steps commanded
    configData->stepsTaken = 0;          

    // Set the initial time were desired theta message was given. It is assigned to be -1 because we don't want its condition to overlap with the call time as we can recieve a message at call time 0.0, where call time is an integer that can't be negative
    configData->previousWrittenTime = 0;  
    configData->firstCall = true;

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
        printf("\n spinningBodyIn.theta %f", spinningBodyIn.theta);
    }

    uint64_t hingedRigidBodyMsgTimeWritten = HingedRigidBodyMsg_C_timeWritten(&configData->spinningBodyInMsg);
    printf("\n timeWritten %" PRIu64, hingedRigidBodyMsgTimeWritten);
    // Check if we have a new message of desired angle to excute the number of steps commanded (no interaption) 
    if (configData->previousWrittenTime <  hingedRigidBodyMsgTimeWritten || configData->firstCall)
    {   
        configData->firstCall = false;
        // Assign the previous time to be the new written time
        configData->previousWrittenTime = hingedRigidBodyMsgTimeWritten;

        // Assign the input theta as the desired theta
        configData->desiredAngle = spinningBodyIn.theta;
        printf("\n In loop: spinningBodyIn.theta %f", spinningBodyIn.theta);

        // Calculate the de,lta angle
        configData->deltaAngle = (configData->desiredAngle) - (configData->currentMotorAngle);
        printf("\n delta angle %f", configData->deltaAngle);

        // Calculate the number of stepps commanded      
        configData->stepsCommanded = (int32_t)(configData->deltaAngle / configData->stepAngle);
        printf("\n stepsCommanded %" PRIi32, configData->stepsCommanded);

        // Output the stepps commanded message to excute the manuevere
        motorStepCountOut.numSteps = (int)configData->stepsCommanded; 

        // Zero steps taken in case of an interaption and start over 
        configData->stepsTaken = 0; 
    } 
        //while the stepps commanded are excuting we need to calculate the steps taken and the steps requested and the time it
        //took to reach steps taken in order to know how many steps are lift to complete the execution

    // Assure that the steps calculated are in the arthematic sequence of our step angle 
    if (configData->stepsCommanded - configData->stepsTaken != 0)
{
    // Calculate the time taken from the start time of the message given to the current time
    configData->deltaSimTime = callTime - configData->previousWrittenTime;
    printf("\n callTime %" PRIu64, callTime);
    printf("\n configData->previousWrittenTime %" PRIu64, configData->previousWrittenTime);
    printf("\n configData->deltaSimTime %" PRIu64, configData->deltaSimTime);

    // Calculate the number of steps already achieved from the steps commanded:
    int32_t localSteps = configData->deltaSimTime / (uint64_t)(configData->stepTime * 1E9);
    printf("\n localSteps %" PRIi32, localSteps);

    // initialize the newStepsTakenSinceLastModuleCallTime to zero
    int32_t newStepsTakenSinceLastModuleCallTime = 0;

    // Update stepsTaken with the new steps, ensuring it doesn't exceed the commanded steps
    //Here if the local step exceeded the steps commanded so we make sure the steps taken are assigned to teps comanded value and for new steps taken we take the diff between steps commanded and steps taken 
    if (localSteps > abs(configData->stepsCommanded)) {
        newStepsTakenSinceLastModuleCallTime = (configData->stepsCommanded - configData->stepsTaken);
        configData->stepsTaken = abs(configData->stepsCommanded);
    //in here we assume everything is fine and still didn't reach the steps commanded, so the steps taken will equal the local steps and the new steps aken will calculate the diff between the local steps and steps taken 
    } else {
        newStepsTakenSinceLastModuleCallTime = localSteps - configData->stepsTaken;
        printf("\n newStepsTakenSinceLastModuleCallTime  %" PRIi32, newStepsTakenSinceLastModuleCallTime);
        configData->stepsTaken = localSteps;
    }

    printf("\n configData->stepsTime %f", configData->stepTime);
    printf("\n configData->stepsTaken %" PRIi32, configData->stepsTaken);

    // Update stepCount with the new steps taken since the last update
    configData->stepCount += newStepsTakenSinceLastModuleCallTime;
    printf("\n configData->stepcount %" PRIi32, configData->stepCount);

    // Update the current angle with each step
    configData->currentMotorAngle = configData->stepCount * configData->stepAngle;
    printf("\n configData->currentMotorAngle %f", configData->currentMotorAngle);
}

    MotorStepCountMsg_C_write(&motorStepCountOut, &configData->motorStepCountOutMsg, moduleID, callTime);
}    