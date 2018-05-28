/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef SPACECRAFT_DYNAMICS_H
#define SPACECRAFT_DYNAMICS_H

#include <vector>
#include <stdint.h>
#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/gravityEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "../_GeneralModuleFiles/sys_model.h"

/*! @brief This is an instantiation of the dynamicObject abstract class that is a spacecraft with stateEffectors and
 dynamicEffectors attached to it. The spacecraftDynamics allows for just translation, just rotation, or both translation and
 rotation. stateEffectors such as RWs, flexible solar panel, fuel slosh etc can be added to the spacecraft by attaching 
 stateEffectors. dynamicEffectors such as thrusters, external force and torque, SRP etc can be added to the spacecraft 
 by attaching dynamicEffectors. This class performs all of this interaction between stateEffectors, dynamicEffectors and
  the hub.

 The module
 [PDF Description](Basilisk-SPACECRAFTPLUS-20170808.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

 */
class SpacecraftDynamics : public DynamicObject{
public:
    uint64_t simTimePrevious;            //!< -- Previous simulation time
    uint64_t numOutMsgBuffers;           //!< -- Number of output message buffers for I/O
    std::string sysTimePropertyName;     //!< -- Name of the system time property
    double currTimeStep;                 //!< [s] Time after integration, used for dvAccum calculation
    double timePrevious;                 //!< [s] Time before integration, used for dvAccum calculation
    Eigen::MatrixXd *sysTime;            //!< [s] System time
    GravityEffector gravField;           //!< -- Gravity effector for gravitational field experienced by spacecraft
    
public:
    SpacecraftDynamics();                    //!< -- Constructor
    ~SpacecraftDynamics();                   //!< -- Destructor
    void initializeDynamics();           //!< -- This method initializes all of the dynamics and variables for the s/c
    void computeEnergyMomentum(double time);  //!< -- This method computes the total energy and momentum of the s/c
    void updateSystemMassProps(double time);  //!< -- This method computes the total mass properties of the s/c
    void SelfInit();                     //!< -- Lets spacecraft plus create its own msgs
    void CrossInit();                    //!< -- Hook to tie s/c plus back into provided msgs
    void writeOutputMessages(uint64_t clockTime); //!< -- Method to write all of the class output messages
    void UpdateState(uint64_t CurrentSimNanos);  //!< -- Runtime hook back into Basilisk arch
    void linkInStates(DynParamManager& statesIn);  //!< Method to get access to the hub's states
    void equationsOfMotion(double integTimeSeconds);    //!< -- This method computes the equations of motion for the whole system
    void integrateState(double time);       //!< -- This method steps the state forward one step in time

private:
    
};

#endif /* SPACECRAFT_DYNAMICS_H */
