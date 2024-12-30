/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "magneticFieldCenteredDipole.h"
#include "architecture/utilities/linearAlgebra.h"

/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.

 */
MagneticFieldCenteredDipole::MagneticFieldCenteredDipole()
{
    //! - Set the default magnetic properties to yield a zero response
    this->g10 = 0.0;            // [T]
    this->g11 = 0.0;            // [T]
    this->h11 = 0.0;            // [T]
    this->planetRadius = 0.0;   // [m]

    return;
}

/*! Empty destructor method.

 */
MagneticFieldCenteredDipole::~MagneticFieldCenteredDipole()
{
    return;
}

/*! This method is evaluates the centered dipole magnetic field model.
 @param msg magnetic field message structure
 @param currentTime current time (s)

 */
void MagneticFieldCenteredDipole::evaluateMagneticFieldModel(MagneticFieldMsgPayload *msg, double currentTime)
{
    Eigen::Vector3d magField_P;         // [T] magnetic field in Planet fixed frame
    Eigen::Vector3d rHat_P;             // [] normalized position vector in E frame components
    Eigen::Vector3d dipoleCoefficients; // [] The first 3 IGRF coefficient that define the magnetic dipole

    //! - compute normalized E-frame position vector
    rHat_P = this->r_BP_P.normalized();

    //! - compute magnetic field vector in E-frame components (see p. 405 in doi:10.1007/978-1-4939-0802-8)
    dipoleCoefficients << this->g11, this->h11, this->g10;
    magField_P = pow(this->planetRadius/this->orbitRadius,3)*(3* rHat_P*rHat_P.dot(dipoleCoefficients) - dipoleCoefficients);

    //! - convert magnetic field vector in N-frame components and store in output message
    m33tMultV3(this->planetState.J20002Pfix, magField_P.data(), msg->magField_N);

    return;
}
