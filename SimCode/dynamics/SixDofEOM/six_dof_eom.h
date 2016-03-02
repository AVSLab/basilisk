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

#ifndef SIX_DOF_EOM_H
#define SIX_DOF_EOM_H

#include <vector>
#include "utilities/sys_model.h"
#include "utilities/dyn_effector.h"
#include "utilities/sphericalHarmonics.h"
#include "utilities/coeffLoader.h"
#include "dynamics/Thrusters/thruster_dynamics.h"
#include "dynamics/ReactionWheels/reactionwheel_dynamics.h"
/*! \addtogroup SimModelGroup
 * @{
 */

//!@brief Container for gravitational body data
/*! This class is designed to hold all of the information for a gravity 
    body.  The nominal use-case has it initialized at the python level and 
    attached to dynamics using the AddGravityBody method.
*/
class GravityBodyData
{
public:
    // TO-DO: modified by MANUEL.
    // This class used to be a structure. For backwards compatibility, all attributes will remain PUBLIC.
    // New attributes are added as Private.
    bool IsCentralBody;             //!<          Flag indicating that object is center
    bool IsDisplayBody;             //!<          Flag indicating that body is display
    bool UseJParams;                //!<          Flag indicating to use perturbations
    bool UseSphericalHarmParams;    //!<          Flag indicating to use spherical harmonics perturbations
    std::vector<double> JParams;    //!<          J perturbations to include
    double PosFromEphem[3];         //!< [m]      Position vector from central to body
    double VelFromEphem[3];         //!< [m/s]    Velocity vector from central body
    double J20002Pfix[3][3];        //!<          Transformation matrix from J2000 to planet-fixed
    double J20002Pfix_dot[3][3];    //!<          Derivative of the transformation matrix from J2000 to planet-fixed
    double posRelDisplay[3];        //!< [m]      Position of planet relative to display frame
    double velRelDisplay[3];        //!< [m]      Velocity of planet relative to display frame
    double mu;                      //!< [m3/s^2] central body gravitational param
    double ephemTime;               //!< [s]      Ephemeris time for the body in question
    double ephIntTime;              //!< [s]      Integration time associated with the ephem data
    double radEquator;              //!< [m]      Equatorial radius for the body
    uint64_t ephemTimeSimNanos;     //!< [ns]     Simulation nanoseconds associated with Ephemeris time
    std::string BodyMsgName;        //!<          Gravitational body name
    std::string outputMsgName;      //!<          Ephemeris information relative to display frame
    std::string planetEphemName;    //!<          Ephemeris name for the planet
    int64_t outputMsgID;            //!<          ID for output message data
    int64_t BodyMsgID;              //!<          ID for ephemeris data message
    
    // Default constructor
    GravityBodyData();
    
    // Constructor to be used for creating bodies with a spherical harmonic model
    GravityBodyData(const std::string& sphHarm_filename, const unsigned int max_degree, const double mu_in, const double reference_radius);
    virtual ~GravityBodyData();
    
    // Copy constructor
    GravityBodyData(const GravityBodyData& gravBody);
    
    // Overloaded operators
    GravityBodyData& operator=(const GravityBodyData& gravBody);
    
    sphericalHarmonics* getSphericalHarmonicsModel(void);
private:
    sphericalHarmonics* _spherHarm;     //!<          Object that computes the spherical harmonics gravity field
    coeffLoaderCSV* _coeff_loader;      //!<          Object that loads the coefficients
};

//!@brief The SixDofEOM class is used to handle all dynamics propagation for a spacecraft
/*! It is designed to handle all gravitational effects and unforced attitude 
    propagation internally.  All non-conservative effects are designed to be 
    handled by the DynEffector class and attached to dynamics through the 
    AddBodyEffector call.
*/
class SixDofEOM: public SysModel {
public:
    SixDofEOM();
    ~SixDofEOM();
    
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void ReadInputs();
    void equationsOfMotion(double t, double *X, double *dX,
                           GravityBodyData *CentralBody);
    void integrateState(double CurrentTime);
    void computeOutputs();
    void AddGravityBody(GravityBodyData *NewBody);
    void WriteOutputMessages(uint64_t CurrentClock);
    void addThrusterSet(ThrusterDynamics *NewEffector);
	void addReactionWheelSet(ReactionWheelDynamics *NewEffector);
    void initPlanetStateMessages();
    void jPerturb(GravityBodyData *gravBody, double r_N[3], double perturbAccel[3]);
    void computeCompositeProperties();
public:
    std::vector<double> PositionInit; //!< [m]   Initial position (inertial)
    std::vector<double> VelocityInit; //!< [m/s] Initial velocity (inertial)
    std::vector<double> AttitudeInit; //!<       Inertial relative MRPs for attitude
    std::vector<double> AttRateInit;  //!< [r/s] Inertial relative body rate
    std::vector<double> baseInertiaInit;//!< [kgm2] Inertia tensor at init (dry)
    std::vector<double> baseCoMInit;  //!< [m]   Initial center of mass in structure (dry)
    std::vector<double> T_Str2BdyInit;//!< []    Initial (perm) structure to bdy rotation
    double ibaseMassInit;             //!< [kg]  Initial mass of vehicle (dry)
    
    std::string OutputStateMessage;   //!<       Output state data
    std::string OutputMassPropsMsg;   //!<       Output mass properties
    uint64_t OutputBufferCount;
    std::vector<GravityBodyData> GravData; //!<  Central body grav information
    bool MessagesLinked;              //!<       Indicator for whether inputs bound
    uint64_t RWACount;                //!<        Number of reaction wheels to model
    double baseCoM[3];                //!< [m]    center of mass of dry spacecraft str
    double baseI[3][3];               //!< [kgm2] Inertia tensor for base spacecraft str
    double baseMass;                  //!< [kg]   Mass of dry spacecraft structure
    double compCoM[3];                //!< [m]    Center of mass of spacecraft in str
    double compI[3][3];               //!< [kgm2] Inertia tensor for vehicle
    double compIinv[3][3];            //!< [m2/kg] inverse of inertia tensor
    double compMass;                  //!< [kg]   Mass of the vehicle
    double TimePrev;                  //!< [s]    Previous update time
    double r_N[3];                    //!< [m]    Current position vector (inertial)
    double v_N[3];                    //!< [m/s]  Current velocity vector (inertial)
    double sigma[3];                  //!<        Current MRPs (inertial)
    double omega[3];                  //!< [r/s]  Current angular velocity (inertial)
    double InertialAccels[3];         //!< [m/s2] Current calculated inertial accels
    double NonConservAccelBdy[3];     //!< [m/s2] Observed non-conservative body accel
    double T_str2Bdy[3][3];           //!<        Structure to body DCM matrix
    double AccumDVBdy[3];             //!< [m/s]  Accumulated DV in body
    uint64_t MRPSwitchCount;          //!<        Count on times we've shadowed
private:
    double *XState;                   //!<        Container for total state
    int64_t StateOutMsgID;            //!<        Output message id for state data
    int64_t MassPropsMsgID;           //!<        Output message id for state data
    uint32_t NStates;                 //!<        Count on states available
    //std::vector<DynEffector*> BodyEffectors;  //!<  Vector of effectors on body
    std::vector<ThrusterDynamics *> thrusters; //!< (-) Vector of thrusters in body
	std::vector<ReactionWheelDynamics *> reactWheels; //!< (-) Vector of thrusters in body
};

/*! @} */

#endif
