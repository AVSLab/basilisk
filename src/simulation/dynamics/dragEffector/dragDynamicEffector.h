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


#ifndef DRAG_DYNAMIC_EFFECTOR_H
#define DRAG_DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include <vector>
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "../../simMessages/atmoPropsSimMsg.h"
#include "../../simFswInterfaceMessages/navAttIntMsg.h"
#include "utilities/avsEigenMRP.h"
#include "utilities/avsEigenSupport.h"
#include "utilities/bskLogging.h"





//! @brief Container for basic drag parameters - the spacecraft's atmosphere-relative velocity, its projected area, and its drag coefficient.
typedef struct {
    double projectedArea;                    //!< m^2   Area of spacecraft projected in velocity direction
    double dragCoeff;                        //!< --  Nondimensional drag coefficient
    Eigen::Vector3d comOffset;               //!< m distance from center of mass to center of projected area
}DragBaseData;


class DragDynamicEffector: public SysModel, public DynamicEffector {
public:
    DragDynamicEffector();
    ~DragDynamicEffector();
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime);
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void cannonballDrag();
    void updateDragDir();
    void setDensityMessage(std::string newDensMessage);

public:
    DragBaseData coreParams;                               //!< -- Struct used to hold drag parameters
    std::string atmoDensInMsgName;                         //!< -- message used to read command inputs
    std::string navAttInMsgName;                         //!< -- message used to read spacecraft attitude
    std::string modelType;                                 //!< -- String used to set the type of model used to compute drag
    StateData *hubSigma;                                   //!< -- Hub/Inertial attitude represented by MRP
    StateData *hubVelocity;                                //!< m/s Hub inertial velocity vector
    Eigen::Vector3d v_B;                         //!< m/s local variable to hold the inertial velocity
    Eigen::Vector3d v_hat_B;                          //!< -- Drag force direction in the inertial frame
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    uint64_t densInMsgId;                            //!< -- Message ID for incoming data
    AtmoPropsSimMsg atmoInData;
    
};


#endif /* THRUSTER_DYNAMIC_EFFECTOR_H */
