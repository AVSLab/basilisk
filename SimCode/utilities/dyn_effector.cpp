
#include "utilities/dyn_effector.h"
#include <cstring>

/*! This is the constructor, zero the public outputs here. */
DynEffector::DynEffector()
{
    memset(BodyForce, 0x0, 3*sizeof(double));
    memset(BodyTorque, 0x0, 3*sizeof(double));
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
