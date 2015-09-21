
#include "utilities/dyn_effector.h"
#include <cstring>

DynEffector::DynEffector()
{
    memset(BodyForce, 0x0, 3*sizeof(double));
    memset(BodyTorque, 0x0, 3*sizeof(double));
}

DynEffector::~DynEffector()
{
}

void DynEffector :: ComputeDynamics(MassPropsData *Props,
                                    OutputStateData *Bstate, double CurrentTime)
{
    return;
}
