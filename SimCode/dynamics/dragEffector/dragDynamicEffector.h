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


#ifndef THRUSTER_DYNAMIC_EFFECTOR_H
#define THRUSTER_DYNAMIC_EFFECTOR_H

#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "../../environment/ExponentialAtmosphere/exponentialAtmosphere.h"
#include <Eigen/Dense>
#include <vector>



/*! \addtogroup SimModelGroup
 * @{
 */

//! @brief Container for basic drag parameters - the spacecraft's atmosphere-relative velocity, its projected area, and its drag coefficient.*/
typedef struct {
    double velocityMag;                 //!< m/s Norm of the atmosphere-relative velocity
    double projectedArea;                    //!< m^2   Area of spacecraft projected in velocity direction
    double dragCoeff;                    //!< --  Nondimensional drag coefficient
    Eigen::Vector3d comOffset     ;               //!< m distance from center of mass to center of projected area
}DragBaseData;

//! @brief Container for current operational data of a given thruster
/*! This structure is used to determine the current state of a given thruster.
 It defines where in the cycle the thruster is and how much longer it should be
 on for.  It is intended to have the previous firing remain resident for logging*/
typedef struct {
    double PreviousIterTime;             //!< s  Previous thruster int time
    double PreviousDragForce;                  //!< N  Total amount of time thruster has fire
}CurrentDragData;

/*! This structure is used in the messaging system to communicate what the
 state of the vehicle is currently.*/
typedef struct {
    Eigen::Vector3d bodyDragForce;                     //!< m  Current position vector (inertial)
    Eigen::Vector3d bodyDragTorque;                    //!< -- Unit vector of thruster pointing
    double maxDrag;                               //!< N  Steady state thrust of thruster
    double maxTorque;                            //!< -- Current Thrust Percentage
}DragOutputData;

//! @brief Thruster dynamics class used to provide thruster effects on body
/*! This class is used to hold and operate a set of thrusters that are located
 on the spacecraft.  It contains all of the configuration data for the thruster
 set, reads an array of on-time requests (double precision in seconds).  It is
 intended to be attached to the dynamics plant in the system using the
 DynEffector interface and as such, does not directly write the current force
 or torque into the messaging system.  The nominal interface to dynamics are the
 dynEffectorForce and dynEffectorTorque arrays that are provided by the DynEffector base class.
 There is technically double inheritance here, but both the DynEffector and
 SysModel classes are abstract base classes so there is no risk of diamond.*/
class DragDynamicEffector: public SysModel, public DynamicEffector {
public:
    DragDynamicEffector();
    ~DragDynamicEffector();
    void linkInStates(DynParamManager& states);
    void computeBodyForceTorque(double integTime);
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void CannonballDrag();
    void PlateDrag();
    void ComputeDragDir();
    void SetArea(double newArea);
    void SetDragCoeff(double newCoeff);
    void SetDensityMessage(std::string newDensMessage);

public:
    DragBaseData coreParams;
    std::string atmoDensInMsgName;                         //!< -- message used to read command inputs
    std::string modelType;
    StateData *hubSigma;
    StateData *hubVelocity;
    CurrentDragData dragHist;
    Eigen::Vector3d locInertialVel;
    AtmoOutputData densityBuffer;

private:
    //    bool bdyFrmReady;                              //!< [-] Flag indicating that the body frame is ready
    Eigen::MatrixXd *dcm_BS;               //!< [kg] spacecrafts total mass
    uint64_t DensInMsgId;                            //!< -- Message ID for incoming data
    AtmoOutputData atmoInData;
    Eigen::Vector3d dragDirection;
};

#endif /* THRUSTER_DYNAMIC_EFFECTOR_H */
