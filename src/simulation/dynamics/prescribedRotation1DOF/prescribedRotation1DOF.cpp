/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "prescribedRotation1DOF.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <cmath>


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedRotation1DOF::Reset(uint64_t callTime)
{
    // Check if the required input message is linked
    if (!this->spinningBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: prescribedRot.spinningBodyInMsg wasn't connected.");
    }

    // Set the initial time
    this->tInit = 0.0;

    // Set the initial convergence to true to enter the required loop in Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedRotation1DOF::UpdateState(uint64_t callTime)
{
    // Create the buffer messages
    HingedRigidBodyMsgPayload spinningBodyIn;
    HingedRigidBodyMsgPayload spinningBodyOut;
    PrescribedRotationMsgPayload prescribedRotationOut;

    // Zero the output messages
    spinningBodyOut = HingedRigidBodyMsgPayload();
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Read the input message
    spinningBodyIn = HingedRigidBodyMsgPayload();
    if (this->spinningBodyInMsg.isWritten()) {
        spinningBodyIn = this->spinningBodyInMsg();
    }

    /* This loop is entered (a) initially and (b) when each attitude maneuver is complete. The reference angle is updated
    even if a new message is not written */
    if (this->spinningBodyInMsg.timeWritten() <= callTime && this->convergence)
    {
        // Store the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Calculate the current ange and angle rate
        double prv_FM_array[3];
        MRP2PRV(this->sigma_FM, prv_FM_array);
        this->thetaInit = v3Dot(prv_FM_array, this->rotAxis_M);
        this->thetaDotInit = v3Norm(this->omega_FM_F);

        // Store the reference angle
        this->thetaRef = spinningBodyIn.theta;

        // Define temporal information for the maneuver
        double convTime = sqrt(((0.5 * fabs(this->thetaRef - this->thetaInit)) * 8) / this->thetaDDotMax);
        this->tf = this->tInit + convTime;
        this->ts = this->tInit + convTime / 2;

        // Define the parabolic constants for the first and second half of the maneuver
        this->a = 0.5 * (this->thetaRef - this->thetaInit) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (this->thetaRef - this->thetaInit) / ((this->ts - this->tf) * (this->ts - this->tf));

        // Set the convergence to false until the attitude maneuver is complete
        this->convergence = false;
    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Define the scalar prescribed states
    double thetaDDot;
    double thetaDot;
    double theta;

    // Compute the prescribed scalar states at the current simulation time
    if ((t < this->ts || t == this->ts) && this->tf - this->tInit != 0) // Entered during the first half of the maneuver
    {
        thetaDDot = this->thetaDDotMax;
        thetaDot = thetaDDot * (t - this->tInit) + this->thetaDotInit;
        theta = this->a * (t - this->tInit) * (t - this->tInit) + this->thetaInit;
    }
    else if ( t > this->ts && t <= this->tf && this->tf - this->tInit != 0) // Entered during the second half of the maneuver
    {
        thetaDDot = -1 * this->thetaDDotMax;
        thetaDot = thetaDDot * (t - this->tInit) + this->thetaDotInit - thetaDDot * (this->tf - this->tInit);
        theta = this->b * (t - this->tf) * (t - this->tf) + this->thetaRef;
    }
    else // Entered when the maneuver is complete
    {
        thetaDDot = 0.0;
        theta = this->thetaRef;
        thetaDot = 0.0;
        this->convergence = true;
    }

    // Determine the prescribed parameters: omega_FM_F and omegaPrime_FM_F
    v3Normalize(this->rotAxis_M, this->rotAxis_M);
    v3Scale(thetaDot, this->rotAxis_M, this->omega_FM_F);
    v3Scale(thetaDDot, this->rotAxis_M, this->omegaPrime_FM_F);

    // Determine dcm_FF0
    double dcm_FF0[3][3];
    double prv_FF0_array[3];
    double theta_FF0 = theta - this->thetaInit;
    v3Scale(theta_FF0, this->rotAxis_M, prv_FF0_array);
    PRV2C(prv_FF0_array, dcm_FF0);

    // Determine dcm_F0M
    double dcm_F0M[3][3];
    double prv_F0M_array[3];
    v3Scale(this->thetaInit, this->rotAxis_M, prv_F0M_array);
    PRV2C(prv_F0M_array, dcm_F0M);

    // Determine dcm_FM
    double dcm_FM[3][3];
    m33MultM33(dcm_FF0, dcm_F0M, dcm_FM);

    // Determine the prescribed parameter: sigma_FM
    C2MRP(dcm_FM, this->sigma_FM);

    // Copy the module variables to the prescribedRotationOut output message
    v3Copy(this->omega_FM_F, prescribedRotationOut.omega_FM_F);
    v3Copy(this->omegaPrime_FM_F, prescribedRotationOut.omegaPrime_FM_F);
    v3Copy(this->sigma_FM, prescribedRotationOut.sigma_FM);

    // Copy the local scalar variables to the spinningBodyOut output message
    spinningBodyOut.theta = theta;
    spinningBodyOut.thetaDot = thetaDot;

    // Write the output messages
    this->spinningBodyOutMsg.write(&spinningBodyOut, moduleID, callTime);
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
}
