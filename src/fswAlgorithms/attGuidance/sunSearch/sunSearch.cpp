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

#include "sunSearch.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>

/*! Module constructor */
SunSearch::SunSearch() = default;


/*! Module destructor */
SunSearch::~SunSearch() = default;


/*! Initialize C-wrapped output messages */
void SunSearch::SelfInit(){
    AttGuidMsg_C_init(&this->attGuidOutMsgC);
}

/*! This method is used to reset the module.
 @return void
 */
void SunSearch::Reset(uint64_t CurrentSimNanos)
{
    this->resetTime = CurrentSimNanos;
}


/*! This method is the main carrier for the computation of the guidance message
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void SunSearch::UpdateState(uint64_t CurrentSimNanos)
{

}


/*! Define this method to compute the kinematic properties of each slew
    @return void
    */
void SunSearch::computeKinematicProperties(int const index)
{
    SlewProperties* SP = &this->slewProperties[index];
    int axis = SP->slewRotAxis - 1;
    double maxAcc = this->slewMaxTorque[axis] / this->principleInertias[axis];

    /*! Computing fastest bang-bang slew with no coasting arc */
    double alpha = 4 * SP->slewAngle / (SP->slewTime * SP->slewTime);
    double omegaMax = 2 * SP->slewAngle / SP->slewTime;
    double totalTime = SP->slewTime;
    double thrustTime = totalTime / 2;

    /*! If angular acceleration exceeds limit, decrease acceleration and increase slew time */
    if (alpha > maxAcc) {
        alpha = maxAcc;
        totalTime = 2 * sqrt(SP->slewAngle / alpha);
        thrustTime = totalTime / 2;
        omegaMax = alpha * thrustTime;
    }

    /*! If angular rate exceeds limit, increase slew time adding a coasting arc */
    if (omegaMax > SP->slewMaxRate) {
        omegaMax = SP->slewMaxRate;
        totalTime = SP->slewAngle / omegaMax + omegaMax / alpha;
        thrustTime = omegaMax / alpha;
    }

    SP->slewAngAcc     = alpha;
    SP->slewOmegaMax   = omegaMax;
    SP->slewTotalTime  = totalTime;
    SP->slewThrustTime = thrustTime;
}


/*! Set the slew time
    @param double slewTime
    @return void
    */
void SunSearch::setSlewTime(double const t1, const double t2, const double t3) {
    this->slewProperties[0].slewTime = t1;
    this->slewProperties[1].slewTime = t2;
    this->slewProperties[2].slewTime = t3;
}

/*! Set the slew angle
    @param double slewAngle
    @return void
    */
void SunSearch::setSlewAngle(double const theta1, double const theta2, double const theta3) {
    this->slewProperties[0].slewAngle = theta1;
    this->slewProperties[1].slewAngle = theta2;
    this->slewProperties[2].slewAngle = theta3;
}

/*! Set the maximum angle rate
    @param double slewMaxRate
    @return void
    */
void SunSearch::setMaxRate(double const omega1, double const omega2, double const omega3) {
    this->slewProperties[0].slewMaxRate = omega1;
    this->slewProperties[1].slewMaxRate = omega2;
    this->slewProperties[2].slewMaxRate = omega3;
}

/*! Set the rotation axis
    @param double slewRotAxis
    @return void
    */
void SunSearch::setRotAxis(int const a1, int const a2, int const a3) {
    this->slewProperties[0].slewRotAxis = a1;
    this->slewProperties[1].slewRotAxis = a2;
    this->slewProperties[2].slewRotAxis = a3;
}

/*! Set the maximum torque
    @param double slewMaxTorque
    @return void
    */
void SunSearch::setMaxTorque(double const u1, double const u2, double const u3) {
    this->slewMaxTorque[0] = u1;
    this->slewMaxTorque[1] = u2;
    this->slewMaxTorque[2] = u3;
}
