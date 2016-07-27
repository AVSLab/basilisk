/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef _DynEffector_HH_
#define _DynEffector_HH_

#include <string>
#include <cstring>
#include <stdint.h>
/*! \addtogroup Sim Utility Group
 * @{
 */

/*! This structure is used in the messaging system to communicate what the mass 
    properties of the vehicle are currently.*/
typedef struct {
    double Mass;                      //!< kg   Current spacecraft mass
    double CoM[3];                    //!< m    Center of mass of spacecraft (relative to struct)
    double InertiaTensor[3*3];       //!< kgm2 Inertia tensor of spacecraft (relative to body)
    double T_str2Bdy[3*3];           //!< -- Transformation from str to body
}MassPropsData;

/*! This structure is used in the messaging system to communicate what the 
    state of the vehicle is currently.*/
typedef struct {
    double r_N[3];                    //!< m  Current position vector (inertial)
    double v_N[3];                    //!< m/s Current velocity vector (inertial)
    double sigma[3];                  //!< -- Current MRPs (inertial)
    double omega[3];                  //!< r/s Current angular velocity (inertial)
    double T_str2Bdy[3][3];           //!< -- Trans from st2bdy Double booked for ease
    double TotalAccumDVBdy[3];        //!< m/s Accumulated DV for simulation
    uint64_t MRPSwitchCount;          //!< -- Number of times that MRPs have switched
}OutputStateData;

/*! This is really an abstract class that has no inherent functionality on its 
    own.  It does actually implement all of the methods so it technically could 
    be used by itself, but that is not the intent.
 */
class DynEffector
{
    
public:
    DynEffector();
    virtual ~DynEffector();
    virtual void ComputeDynamics(MassPropsData *Props, OutputStateData *Bstate,
                                 double CurrentTime);
    double *GetBodyForces() {return this->dynEffectorForce_B;}
    double *GetBodyTorques() {return this->dynEffectorTorque_B;}
    void getProps(MassPropsData *callerProps)
        {memcpy(callerProps, &objProps, sizeof(MassPropsData));}
    
public:
    double dynEffectorForce_B[3];     //!< N Modeled force on the body
    double dynEffectorTorque_B[3];    //!< Nm Modeled Torque on the body
    MassPropsData objProps;           //!< (-) Update-driven mass properties for object
};

/*! @} */

#endif /* _SYS_MODEL_H_ */
