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

#include "prescribedTranslation.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedTranslation::Reset(uint64_t callTime)
{
    if (!this->linearTranslationRigidBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: prescribedTrans.linearTranslationRigidBodyInMsg wasn't connected.");
    }

    // Set the initial time
    this->tInit = 0.0;

    this->transPos = this->transPosInit;
    this->transVelInit = 0.0;
    this->transVel = 0.0;

    // Set the initial convergence to true to enter the correct loop in the Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedTranslation::UpdateState(uint64_t callTime)
{
    // Create the buffer messages
    LinearTranslationRigidBodyMsgPayload linearTranslationRigidBodyIn;
    PrescribedTranslationMsgPayload prescribedTranslationMsgOut;

    // Zero the output message
    prescribedTranslationMsgOut = PrescribedTranslationMsgPayload();

    // Read the input message
    linearTranslationRigidBodyIn = LinearTranslationRigidBodyMsgPayload();
    if (this->linearTranslationRigidBodyInMsg.isWritten())
    {
        linearTranslationRigidBodyIn = this->linearTranslationRigidBodyInMsg();
    }

    /* This loop is entered (a) initially and (b) when each translation is complete. The reference position is updated
    even if a new message is not written */
    if (this->linearTranslationRigidBodyInMsg.timeWritten() <= callTime && this->convergence)
    {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Store the reference scalar position
        this->transPosRef = linearTranslationRigidBodyIn.rho;

        // Update the initial scalar position
        this->transPosInit = this->transPos;

        // Set the convergence to false until the translation is complete
        this->convergence = false;

        // Set the parameters required to profile the translation
        // Determine the total time required for the translation
        double totalTransTime = sqrt(((0.5 * fabs(this->transPosRef - this->transPosInit)) * 8) / this->transAccelMax);

        // Determine the time at the end of the translation
        this->tf = this->tInit + totalTransTime;

        // Determine the time halfway through the translation
        this->ts = this->tInit + (totalTransTime / 2);

        // Define the parabolic constants for the first and second half of the translation
        this->a = 0.5 * (this->transPosRef - this->transPosInit) /
                        ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (this->transPosRef - this->transPosInit) /
                        ((this->ts - this->tf) * (this->ts - this->tf));

    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Compute the scalar translation states at the current simulation time
    if (t <= this->ts && this->tf - this->tInit != 0) { // Entered during the first half of the translation
        if (this->transPosInit < this->transPosRef) {
            this->transAccel = this->transAccelMax;
        } else {
            this->transAccel = - this->transAccelMax;
        }
        this->transVel = this->transAccel * (t - this->tInit) + this->transVelInit;
        this->transPos = this->a * (t - this->tInit) * (t - this->tInit)
                             + this->transPosInit;
    } else if ( t > this->ts && t <= this->tf && this->tf - this->tInit != 0) { // Entered during the second half of the translation
        if (this->transPosInit < this->transPosRef) {
            this->transAccel = - this->transAccelMax;
        } else {
            this->transAccel = this->transAccelMax;
        }
        this->transVel = this->transAccel * (t - this->tInit) + this->transVelInit
                             - this->transAccel * (this->tf - this->tInit);
        this->transPos = this->b * (t - this->tf) * (t - this->tf) + this->transPosRef;
    } else { // Entered when the translation is complete
        this->transAccel = 0.0;
        this->transVel = 0.0;
        this->transPos = this->transPosRef;
        this->convergence = true;
    }

    // Determine the prescribed parameters: r_FM_M, rPrime_FM_M and rPrimePrime_FM_M
    v3Scale(this->transPos, this->transAxis_M, this->r_FM_M);
    v3Scale(this->transVel, this->transAxis_M, this->rPrime_FM_M);
    v3Scale(this->transAccel, this->transAxis_M, this->rPrimePrime_FM_M);

    // Copy the required module variables to the prescribedTranslation output message
    v3Copy(this->r_FM_M, prescribedTranslationMsgOut.r_FM_M);
    v3Copy(this->rPrime_FM_M, prescribedTranslationMsgOut.rPrime_FM_M);
    v3Copy(this->rPrimePrime_FM_M, prescribedTranslationMsgOut.rPrimePrime_FM_M);

    // Write the output message
    this->prescribedTranslationOutMsg.write(&prescribedTranslationMsgOut, moduleID, callTime);
}
