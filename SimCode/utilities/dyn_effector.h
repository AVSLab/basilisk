
#ifndef _DynEffector_HH_
#define _DynEffector_HH_

#include <string>
#include <stdint.h>

typedef struct {
    double Mass;                      /// kg   Current spacecraft mass
    double CoM[3];                    /// m    Center of mass of spacecraft
    double InertiaTensor[3][3];       /// kgm2 Inertia tensor of spacecraft
    double T_str2Bdy[3][3];           /// -- Transformation from str to body
}MassPropsData;

typedef struct {
    double r_N[3];                    /// m  Current position vector (inertial)
    double v_N[3];                    /// m/s Current velocity vector (inertial)
    double sigma[3];                  /// -- Current MRPs (inertial)
    double omega[3];                  /// r/s Current angular velocity (inertial)
    double T_str2Bdy[3][3];           /// -- Trans from st2bdy Double booked for ease
    double TotalAccumDVBdy[3];        /// m/s Accumulated DV for simulation
    uint64_t MRPSwitchCount;          /// -- Number of times that MRPs have switched
}OutputStateData;

class DynEffector
{
    
public:
    DynEffector();
    virtual ~DynEffector();
    virtual void ComputeDynamics(MassPropsData *Props, OutputStateData *Bstate,
                                 double CurrentTime);
    double *GetBodyForces() {return BodyForce;}
    double *GetBodyTorques() {return BodyTorque;}
    
public:
    double BodyForce[3];              /// N Modeled force on the body
    double BodyTorque[3];             /// Nm Modeled Torque on the body
};


#endif /* _SYS_MODEL_H_ */
