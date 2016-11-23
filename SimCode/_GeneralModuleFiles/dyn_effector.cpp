/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "_GeneralModuleFiles/dyn_effector.h"
#include <cstring>

/*! This is the constructor, zero the public outputs here. */
DynEffector::DynEffector()
{
    memset(this->dynEffectorForce_N, 0x0, 3*sizeof(double));
    memset(this->dynEffectorForce_B, 0x0, 3*sizeof(double));
    memset(this->dynEffectorTorquePntB_B, 0x0, 3*sizeof(double));
}

/*! Nothing to destroy */
DynEffector::~DynEffector()
{
}

/*! This method is the main interface for children of the DynEffector class.  
    It does nothing in the base class, but the intent is that child classes will 
    make use of this method extensively.
    @return void
    @param Props Current mass properties of the vehicle
    @param Bstate The current body state for the vehicle
    @param CurrentTime The call time computed by the upper level dynamics routine
*/
void DynEffector :: ComputeDynamics(MassPropsData *Props,
                                    OutputStateData *Bstate, double CurrentTime)
{
    return;
}
